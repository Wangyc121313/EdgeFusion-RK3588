#include "pipeline/perception_pipeline.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <linux/videodev2.h>

#include "camera/camera_capture.hpp"
#include "core/bounded_queue.hpp"
#include "core/data_types.hpp"
#include "core/lidar_ring_buffer.hpp"
#include "fusion/detection_distance_fusion.hpp"
#include "fusion/learned_fusion_corrector.hpp"
#include "fusion/multi_target_tracker.hpp"
#include "fusion/sensor_fusion.hpp"
#include "pipeline/calibration_profile.hpp"
#include "pipeline/pseudo_label_sink.hpp"
#include "pipeline/runtime_telemetry.hpp"
#include "infer/rknn_runner.hpp"
#include "lidar/lidar_adapter.hpp"
#include "lidar/lidar_reader.hpp"
#include "video/mpp_encoder.hpp"
#include "video/nv12_overlay.hpp"
#include "video/rga_processor.hpp"
#include "video/webrtc_publisher.hpp"
#include "video/zlm_rtsp_publisher.hpp"

namespace rk3588::modules {

namespace {

bool isPacked422(std::uint32_t fourcc) {
	return fourcc == V4L2_PIX_FMT_YUYV || fourcc == V4L2_PIX_FMT_YVYU ||
		   fourcc == V4L2_PIX_FMT_UYVY || fourcc == V4L2_PIX_FMT_VYUY;
}

const char* fourccName(std::uint32_t fourcc) {
	switch (fourcc) {
		case V4L2_PIX_FMT_YUYV:
			return "YUYV";
		case V4L2_PIX_FMT_YVYU:
			return "YVYU";
		case V4L2_PIX_FMT_UYVY:
			return "UYVY";
		case V4L2_PIX_FMT_VYUY:
			return "VYUY";
		case V4L2_PIX_FMT_NV12:
			return "NV12";
		default:
			return "UNKNOWN";
	}
}

std::uint64_t nowSteadyMs() {
	return static_cast<std::uint64_t>(
		std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch())
			.count());
}

float wrapSignedAngleDeg(float angle_deg) {
	float wrapped = std::fmod(angle_deg, 360.0F);
	if (wrapped <= -180.0F) {
		wrapped += 360.0F;
	} else if (wrapped > 180.0F) {
		wrapped -= 360.0F;
	}
	return wrapped;
}

std::string formatDetectionBrief(const YoloDetection& det) {
	char buf[192] = {0};
	const int width = std::max(0, det.right - det.left);
	const int height = std::max(0, det.bottom - det.top);
	std::snprintf(buf,
				  sizeof(buf),
				  "top:%s conf=%.2f box=%dx%d",
				  det.class_name.empty() ? "obj" : det.class_name.c_str(),
				  det.confidence,
				  width,
				  height);
	return std::string(buf);
}

std::vector<std::string> buildHudLines(std::uint64_t frame_id,
									   std::uint64_t input_frames,
									   std::uint64_t output_packets,
									   std::uint64_t encode_frames,
									   int infer_every_n_frames,
									   bool did_infer,
									   const std::vector<YoloDetection>& detections,
									   std::uint64_t lidar_scans,
									   std::uint64_t lidar_delta_ms,
									   std::uint32_t pixel_format,
									   std::uint32_t actual_422_fourcc,
									   const char* stream_url,
									   double elapsed_sec) {
	if (!stream_url) {
		stream_url = "n/a";
	}
	std::vector<std::string> lines;
	lines.reserve(5);

	const double encode_fps = elapsed_sec > 1e-3 ? static_cast<double>(encode_frames) / elapsed_sec : 0.0;

	char line0[192] = {0};
	std::snprintf(line0,
				  sizeof(line0),
				  "frame=%llu enc_fps=%.1f infer_n=%d",
				  static_cast<unsigned long long>(frame_id),
				  encode_fps,
				  infer_every_n_frames);
	lines.emplace_back(line0);

	char line1[192] = {0};
	std::snprintf(line1,
				  sizeof(line1),
				  "det=%zu infer=%s in=%llu out=%llu lidar_dt=%llums",
				  detections.size(),
				  did_infer ? "yes" : "reuse",
				  static_cast<unsigned long long>(input_frames),
				  static_cast<unsigned long long>(output_packets),
				  static_cast<unsigned long long>(lidar_delta_ms));
	lines.emplace_back(line1);

	char line1b[192] = {0};
	std::snprintf(line1b,
				  sizeof(line1b),
				  "lidar_scans=%llu",
				  static_cast<unsigned long long>(lidar_scans));
	lines.emplace_back(line1b);

	char line2[192] = {0};
	const std::uint32_t fmt = actual_422_fourcc != 0 ? actual_422_fourcc : pixel_format;
	std::snprintf(line2,
				  sizeof(line2),
				  "fmt=%s stream=%s",
				  fourccName(fmt),
				  stream_url == nullptr ? "n/a" : stream_url);
	lines.emplace_back(line2);

	if (!detections.empty()) {
		lines.emplace_back(formatDetectionBrief(detections.front()));
	}

	return lines;
}

std::vector<TelemetryTarget> buildTelemetryTargets(const SensorFusion& fusion,
									 const std::vector<YoloDetection>& detections,
									 const std::vector<TrackEstimate>* tracks,
									 const std::vector<DistanceFusionDiagnostics>* diagnostics = nullptr,
									 std::size_t max_targets = 8) {
	std::vector<TelemetryTarget> targets;
	const std::size_t count = std::min(max_targets, detections.size());
	targets.reserve(count);
	for (std::size_t i = 0; i < count; ++i) {
		const auto& det = detections[i];
		const TrackEstimate* track = (tracks != nullptr && i < tracks->size()) ? &((*tracks)[i]) : nullptr;
		const DistanceFusionDiagnostics* diag =
			(diagnostics != nullptr && i < diagnostics->size()) ? &((*diagnostics)[i]) : nullptr;
		TelemetryTarget target;
		target.track_id = track != nullptr ? track->track_id : -1;
		target.track_age_frames = track != nullptr ? track->age_frames : 0;
		target.track_idle_frames = track != nullptr ? track->idle_frames : 0;
		target.track_confirmed = track != nullptr ? track->confirmed : false;
		target.track_is_ghost = track != nullptr ? track->is_ghost : false;
		target.class_id = det.class_id;
		target.class_name = det.class_name;
		target.confidence = det.confidence;
		const float center_x = 0.5F * static_cast<float>(det.left + det.right);
		target.angle_deg = fusion.pixelToCameraAngle(center_x);
		target.distance_m = det.distance_m;
		target.lateral_offset_m = track != nullptr ? track->lateral_offset_m : 0.0F;
		target.radial_velocity_mps = track != nullptr ? track->radial_velocity_mps : 0.0F;
		target.lateral_velocity_mps = track != nullptr ? track->lateral_velocity_mps : 0.0F;
		target.closing_speed_mps = track != nullptr ? track->closing_speed_mps : 0.0F;
		target.ttc_s = track != nullptr ? track->ttc_s : -1.0F;
		target.raw_distance_m = diag != nullptr ? diag->raw_distance_m : det.distance_m;
		target.support_points = diag != nullptr ? diag->candidate_points : 0;
		target.cluster_points = diag != nullptr ? diag->cluster_points : 0;
		target.cluster_score = diag != nullptr ? diag->cluster_score : 0.0F;
		target.used_fallback = diag != nullptr ? diag->used_fallback : false;
		target.used_temporal_smoothing = diag != nullptr ? diag->used_temporal_smoothing : false;
		target.rejected_by_sanity = diag != nullptr ? diag->rejected_by_sanity : false;
		target.left = det.left;
		target.top = det.top;
		target.right = det.right;
		target.bottom = det.bottom;
		targets.push_back(std::move(target));
	}
	return targets;
}

std::vector<TelemetryLidarPoint> buildTelemetryLidarPoints(const SensorFusion& fusion,
										 const rk3588::core::PointCloudPacket* cloud,
										 std::size_t max_points = 160) {
	std::vector<TelemetryLidarPoint> points;
	if (cloud == nullptr || cloud->points.empty()) {
		return points;
	}

	points.reserve(std::min(max_points, cloud->points.size()));
	for (const auto& point : cloud->points) {
		const float camera_angle_deg = fusion.lidarAngleToCamera(point.angle_deg);
		const float radians = camera_angle_deg * static_cast<float>(M_PI) / 180.0F;
		const float x_m = point.distance_m * std::sin(radians);
		const float z_m = point.distance_m * std::cos(radians);
		if (point.distance_m <= 0.01F) {
			continue;
		}

		TelemetryLidarPoint telemetry_point;
		telemetry_point.angle_deg = camera_angle_deg;
		telemetry_point.distance_m = point.distance_m;
		telemetry_point.x_m = x_m;
		telemetry_point.z_m = z_m;
		points.push_back(telemetry_point);
	}

	if (points.size() > max_points) {
		std::vector<TelemetryLidarPoint> reduced;
		reduced.reserve(max_points);
		const double step = static_cast<double>(points.size()) / static_cast<double>(max_points);
		for (std::size_t i = 0; i < max_points; ++i) {
			const std::size_t index = std::min(points.size() - 1,
				static_cast<std::size_t>(std::floor(i * step)));
			reduced.push_back(points[index]);
		}
		return reduced;
	}

	return points;
}

void decode422Pair(std::uint32_t fourcc,
				   const std::uint8_t* src4,
				   std::uint8_t* y0,
				   std::uint8_t* y1,
				   std::uint8_t* u,
				   std::uint8_t* v) {
	switch (fourcc) {
		case V4L2_PIX_FMT_YUYV:
			*y0 = src4[0];
			*u = src4[1];
			*y1 = src4[2];
			*v = src4[3];
			break;
		case V4L2_PIX_FMT_YVYU:
			*y0 = src4[0];
			*v = src4[1];
			*y1 = src4[2];
			*u = src4[3];
			break;
		case V4L2_PIX_FMT_UYVY:
			*u = src4[0];
			*y0 = src4[1];
			*v = src4[2];
			*y1 = src4[3];
			break;
		case V4L2_PIX_FMT_VYUY:
			*v = src4[0];
			*y0 = src4[1];
			*u = src4[2];
			*y1 = src4[3];
			break;
		default:
			*y0 = 0;
			*y1 = 0;
			*u = 0;
			*v = 0;
			break;
	}
}

std::uint32_t detectPacked422Fourcc(const std::uint8_t* src,
									std::uint32_t width,
									std::uint32_t height,
									std::uint32_t src_stride) {
	if (src == nullptr || width < 2 || height == 0) {
		return V4L2_PIX_FMT_YUYV;
	}

	const std::uint32_t in_stride = src_stride >= width * 2 ? src_stride : width * 2;
	const std::uint32_t candidates[] = {
		V4L2_PIX_FMT_YUYV,
		V4L2_PIX_FMT_YVYU,
		V4L2_PIX_FMT_UYVY,
		V4L2_PIX_FMT_VYUY,
	};

	double best_score = -1e30;
	std::uint32_t best_fourcc = V4L2_PIX_FMT_YUYV;

	const std::uint32_t rows = std::min<std::uint32_t>(height, 64);
	const std::uint32_t cols = std::min<std::uint32_t>(width, 640);

	for (const auto fourcc : candidates) {
		double sum_y = 0.0;
		double sum_yy = 0.0;
		double sum_u = 0.0;
		double sum_uu = 0.0;
		double sum_v = 0.0;
		double sum_vv = 0.0;
		std::uint64_t n = 0;

		for (std::uint32_t y = 0; y < rows; ++y) {
			const std::uint8_t* row = src + static_cast<std::size_t>(y) * in_stride;
			for (std::uint32_t x = 0; x + 1 < cols; x += 2) {
				std::uint8_t y0 = 0;
				std::uint8_t y1 = 0;
				std::uint8_t u = 0;
				std::uint8_t v = 0;
				decode422Pair(fourcc, row + x * 2, &y0, &y1, &u, &v);

				sum_y += static_cast<double>(y0) + static_cast<double>(y1);
				sum_yy += static_cast<double>(y0) * y0 + static_cast<double>(y1) * y1;
				sum_u += static_cast<double>(u);
				sum_uu += static_cast<double>(u) * u;
				sum_v += static_cast<double>(v);
				sum_vv += static_cast<double>(v) * v;
				++n;
			}
		}

		if (n == 0) {
			continue;
		}

		const double y_count = static_cast<double>(n) * 2.0;
		const double mean_y = sum_y / y_count;
		const double mean_u = sum_u / static_cast<double>(n);
		const double mean_v = sum_v / static_cast<double>(n);
		const double var_y = (sum_yy / y_count) - mean_y * mean_y;
		const double var_u = (sum_uu / static_cast<double>(n)) - mean_u * mean_u;
		const double var_v = (sum_vv / static_cast<double>(n)) - mean_v * mean_v;

		const double score = var_y - 0.6 * (var_u + var_v) -
							 0.08 * (std::abs(mean_u - 128.0) + std::abs(mean_v - 128.0));
		if (score > best_score) {
			best_score = score;
			best_fourcc = fourcc;
		}
	}

	return best_fourcc;
}

bool convertPacked422ToNv12(const std::uint8_t* src,
							std::uint32_t width,
							std::uint32_t height,
							std::uint32_t src_stride,
							std::uint32_t src_fourcc,
							std::uint32_t nv12_stride,
							bool swap_uv,
							std::vector<std::uint8_t>* nv12_out) {
	if (src == nullptr || nv12_out == nullptr || width == 0 || height == 0) {
		return false;
	}
	if ((width & 1U) != 0U || (height & 1U) != 0U || !isPacked422(src_fourcc)) {
		return false;
	}

	const std::uint32_t in_stride = src_stride >= width * 2 ? src_stride : width * 2;
	const std::uint32_t out_stride = nv12_stride >= width ? nv12_stride : width;
	const std::size_t out_size = static_cast<std::size_t>(out_stride) * height * 3 / 2;
	nv12_out->assign(out_size, 0);

	std::uint8_t* dst_y = nv12_out->data();
	std::uint8_t* dst_uv = dst_y + static_cast<std::size_t>(out_stride) * height;

	for (std::uint32_t y = 0; y < height; ++y) {
		const std::uint8_t* src_row = src + static_cast<std::size_t>(y) * in_stride;
		std::uint8_t* y_row = dst_y + static_cast<std::size_t>(y) * out_stride;
		std::uint8_t* uv_row = dst_uv + static_cast<std::size_t>(y / 2) * out_stride;

		for (std::uint32_t x = 0; x < width; x += 2) {
			std::uint8_t y0 = 0;
			std::uint8_t y1 = 0;
			std::uint8_t u = 0;
			std::uint8_t v = 0;
			decode422Pair(src_fourcc, src_row + x * 2, &y0, &y1, &u, &v);

			if (swap_uv) {
				std::swap(u, v);
			}

			y_row[x] = y0;
			y_row[x + 1] = y1;

			if ((y & 1U) == 0U) {
				uv_row[x] = u;
				uv_row[x + 1] = v;
			}
		}
	}

	return true;
}

}  // namespace

PerceptionPipeline::PerceptionPipeline(AppConfig config) : config_(std::move(config)) {}

int PerceptionPipeline::run() {
	if (!config_.publishModeValid()) {
		std::cerr << "invalid publish_mode=" << config_.publish_mode << ", expected rtsp|webrtc|both\n";
		return 1;
	}

	if (!config_.calibration_profile_path.empty()) {
		CalibrationProfile profile;
		if (loadCalibrationProfile(config_.calibration_profile_path, &profile)) {
			applyCalibrationProfile(profile, &config_);
			std::cout << "loaded calibration profile: " << config_.calibration_profile_path
					  << " camera_fov_deg=" << config_.camera_fov_deg
					  << " offset_deg=" << config_.lidar_offset_deg
					  << " overlap_fov_deg=" << config_.lidar_fov_deg
					  << " window_half_deg=" << config_.lidar_window_half_deg
					  << " max_age_ms=" << config_.lidar_max_age_ms << '\n';
		} else {
			std::cerr << "warning: failed to load calibration profile: " << config_.calibration_profile_path << '\n';
		}
	}

	rk3588::core::BoundedQueue<rk3588::core::FramePacket> queue(config_.camera_queue_depth);
	rk3588::core::LidarRingBuffer lidar_buffer;
	CameraCapture camera;
	MPPEncoder encoder;
	RGAProcessor rga;
	RKNNRunner rknn;
	ZlmRtspPublisher streamer;
	WebRtcPublisher webrtc_publisher(config_.webrtc_url);
	RuntimeTelemetrySink telemetry(config_.telemetry_path, config_.telemetry_interval_ms);
	PseudoLabelSink pseudo_labels(config_.pseudo_label_path,
						 config_.pseudo_label_max_lines,
						 config_.pseudo_label_sequence_id,
						 static_cast<std::uint32_t>(std::max(1, config_.fps)));

	FusionConfig fusion_cfg;
	fusion_cfg.image_width = static_cast<int>(config_.camera_width);
	fusion_cfg.image_height = static_cast<int>(config_.camera_height);
	fusion_cfg.camera_fov_deg = config_.camera_fov_deg;
	fusion_cfg.lidar_angle_offset_deg = config_.lidar_offset_deg;
	fusion_cfg.default_window_half_deg = config_.lidar_window_half_deg;
	fusion_cfg.min_valid_distance_m = config_.lidar_min_dist_m;
	SensorFusion fusion(fusion_cfg);
	DistanceFusionConfig distance_fusion_cfg;
	distance_fusion_cfg.use_robust_estimator = config_.distance_fusion_mode != "legacy";
	distance_fusion_cfg.min_distance_m = config_.lidar_min_dist_m;
	distance_fusion_cfg.max_distance_m = config_.lidar_max_dist_m;
	distance_fusion_cfg.window_half_deg = config_.lidar_window_half_deg;
	DetectionDistanceFusion distance_fusion(fusion, distance_fusion_cfg);
	LearnedFusionConfig mlp_cfg;
	{
		const char* mlp_env = std::getenv("RK3588_FUSION_MLP_ENABLED");
		mlp_cfg.enabled = mlp_env != nullptr && mlp_env[0] != '\0' && mlp_env[0] != '0';
	}
	if (mlp_cfg.enabled) {
		const char* blend_env = std::getenv("RK3588_FUSION_MLP_BLEND");
		if (blend_env != nullptr && blend_env[0] != '\0') {
			mlp_cfg.blend_weight = std::max(0.0F, std::min(1.0F, static_cast<float>(std::atof(blend_env))));
		}
		const char* score_env = std::getenv("RK3588_FUSION_MLP_MIN_SCORE");
		if (score_env != nullptr && score_env[0] != '\0') {
			mlp_cfg.min_cluster_score = static_cast<float>(std::atof(score_env));
		}
		const char* pts_env = std::getenv("RK3588_FUSION_MLP_MIN_POINTS");
		if (pts_env != nullptr && pts_env[0] != '\0') {
			mlp_cfg.min_candidate_points = std::max(1, std::atoi(pts_env));
		}
		std::cout << "mlp_fusion enabled: blend=" << mlp_cfg.blend_weight
		          << " min_score=" << mlp_cfg.min_cluster_score
		          << " min_points=" << mlp_cfg.min_candidate_points << '\n';
	}
	LearnedFusionCorrector mlp_corrector(fusion, mlp_cfg);
	std::cout << "distance_fusion_mode="
			  << (distance_fusion_cfg.use_robust_estimator ? "robust" : "legacy") << '\n';
	MultiTargetTrackerConfig tracker_cfg;
	tracker_cfg.max_center_delta_px = std::max(72.0F, static_cast<float>(config_.camera_width) * 0.16F);
	tracker_cfg.min_iou_for_match = config_.tracker_min_iou_for_match;
	tracker_cfg.iou_cost_weight = config_.tracker_iou_cost_weight;
	tracker_cfg.center_velocity_alpha = config_.tracker_center_velocity_alpha;
	tracker_cfg.ghost_velocity_decay = config_.tracker_ghost_velocity_decay;
	tracker_cfg.ghost_keep_frames = config_.tracker_ghost_keep_frames;
	tracker_cfg.max_idle_frames = config_.tracker_max_idle_frames;
	MultiTargetTracker tracker(tracker_cfg);
	std::cout << "tracker_cfg: min_iou=" << tracker_cfg.min_iou_for_match
			  << " iou_weight=" << tracker_cfg.iou_cost_weight
			  << " ghost_keep=" << tracker_cfg.ghost_keep_frames
			  << " max_idle=" << tracker_cfg.max_idle_frames << '\n';
	LidarReader lidar_reader;
	LidarAdapter lidar_adapter;
	LidarReader::Config lidar_cfg;
	lidar_cfg.port = config_.lidar_port.c_str();
	lidar_cfg.baud = config_.lidar_baud;
	lidar_cfg.min_distance_m = config_.lidar_min_dist_m;
	lidar_cfg.max_distance_m = config_.lidar_max_dist_m;

	std::atomic<bool> lidar_running {true};
	std::atomic<std::uint64_t> lidar_scan_count {0};
	std::thread lidar_thread([&] {
		if (!lidar_reader.start(lidar_cfg)) {
			return;
		}

		std::cout << "lidar fusion enabled: port=" << config_.lidar_port
				  << " camera_fov_deg=" << config_.camera_fov_deg
				  << " offset_deg=" << config_.lidar_offset_deg
				  << " overlap_fov_deg=" << config_.lidar_fov_deg
				  << " window_half_deg=" << config_.lidar_window_half_deg
				  << " min_dist=" << config_.lidar_min_dist_m
				  << " max_dist=" << config_.lidar_max_dist_m
				  << " max_age_ms=" << config_.lidar_max_age_ms << '\n';

		while (lidar_running.load()) {
			auto cloud = lidar_reader.poll(200U);
			if (!cloud.has_value()) {
				continue;
			}

			auto adapted = lidar_adapter.adapt(*cloud);
			if (!adapted.points.empty()) {
				lidar_buffer.write(std::move(adapted));
				lidar_scan_count.fetch_add(1);
			}
		}

		lidar_reader.stop();
	});

	const auto stop_lidar = [&] {
		lidar_running = false;
		if (lidar_thread.joinable()) {
			lidar_thread.join();
		}
	};

	if (!camera.init(config_.camera_device,
					 config_.camera_width,
					 config_.camera_height,
					 config_.camera_buffer_count) ||
		!camera.start(&queue)) {
		std::cerr << "camera init/start failed\n";
		stop_lidar();
		return 1;
	}

	bool initialized = false;
	std::uint64_t input_frames = 0;
	std::uint64_t output_packets = 0;
	std::uint64_t encode_frames = 0;
	std::vector<std::uint8_t> rgb;
	std::vector<std::uint8_t> nv12_scratch;
	std::vector<YoloDetection> last_detections;
	std::vector<DistanceFusionDiagnostics> fusion_diagnostics;
	std::vector<TrackEstimate> track_estimates;
	std::uint32_t actual_422_fourcc = 0;
	bool logged_rga_422_fallback = false;
	FILE* dump_file = nullptr;

	if (!config_.dump_h264_path.empty()) {
		dump_file = std::fopen(config_.dump_h264_path.c_str(), "wb");
		if (dump_file == nullptr) {
			std::cerr << "failed to open dump file: " << config_.dump_h264_path << '\n';
		} else {
			std::cout << "dumping h264 stream to " << config_.dump_h264_path << '\n';
		}
	}

	int exit_code = 0;
	const auto start = std::chrono::steady_clock::now();
	while (true) {
		const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
			std::chrono::steady_clock::now() - start).count();
		if (config_.run_seconds > 0 && elapsed >= config_.run_seconds) {
			break;
		}

		rk3588::core::FramePacket frame;
		if (!queue.pop_for(frame, std::chrono::milliseconds(200))) {
			continue;
		}

		if (!initialized) {
			const std::uint32_t gop = static_cast<std::uint32_t>(std::max(10, config_.fps / 2));
			if (!encoder.init() ||
				!encoder.configure(frame.width,
								   frame.height,
								   static_cast<std::uint32_t>(config_.fps),
								   2U * 1024U * 1024U,
								   gop)) {
				std::cerr << "mpp encoder init/config failed\n";
				camera.requeueBuffer(frame.buffer_index);
				exit_code = 1;
				break;
			}

			if (!rga.init(static_cast<int>(frame.width),
						  static_cast<int>(frame.height),
						  static_cast<int>(config_.model_width),
						  static_cast<int>(config_.model_height))) {
				std::cerr << "rga init failed\n";
				camera.requeueBuffer(frame.buffer_index);
				exit_code = 1;
				break;
			}

			if (!rknn.init(config_.model_path,
						   static_cast<int>(config_.model_width),
						   static_cast<int>(config_.model_height),
						   config_.labels_path)) {
				std::cerr << "rknn init failed\n";
				camera.requeueBuffer(frame.buffer_index);
				exit_code = 1;
				break;
			}

			std::cout << "camera format fourcc=0x" << std::hex << frame.pixel_format << std::dec
					  << " infer_every_n_frames=" << config_.infer_every_n_frames
					  << " swap_uv=" << (config_.swap_uv ? "1" : "0")
					  << " gop=" << gop;
			if (config_.forced_422_fourcc != 0) {
				std::cout << " force422=" << fourccName(config_.forced_422_fourcc);
			}
			std::cout << '\n';

			if (config_.rtspEnabled()) {
				if (!streamer.start(config_.rtsp_url,
									frame.width,
									frame.height,
									static_cast<std::uint32_t>(config_.fps))) {
					std::cerr << "zlm rtsp publisher start failed, url=" << config_.rtsp_url << '\n';
					camera.requeueBuffer(frame.buffer_index);
					exit_code = 1;
					break;
				}
				std::cout << "streaming h264 to " << streamer.publishUrl() << " fps=" << config_.fps << '\n';
			}

			if (config_.webrtcEnabled()) {
				webrtc_publisher.setVideoConfig(frame.width, frame.height, static_cast<std::uint32_t>(config_.fps));
				if (!webrtc_publisher.start()) {
					std::cerr << "webrtc publisher start failed, url=" << config_.webrtc_url << '\n';
					camera.requeueBuffer(frame.buffer_index);
					exit_code = 1;
					break;
				}
				std::cout << "streaming h264 to " << webrtc_publisher.rtcPlayUrl()
						  << " sdp_api=" << webrtc_publisher.sdpApiUrl()
						  << " fps=" << config_.fps << '\n';
			}

			std::vector<std::uint8_t> codec_header;
			if (encoder.getCodecHeader(&codec_header) && !codec_header.empty()) {
				if (dump_file != nullptr) {
					(void)std::fwrite(codec_header.data(), 1, codec_header.size(), dump_file);
					std::fflush(dump_file);
				}
				if (config_.rtspEnabled() &&
					!streamer.pushPacket(codec_header.data(), codec_header.size(), 0, 0)) {
					std::cerr << "failed to push codec header to rtsp streamer\n";
					camera.requeueBuffer(frame.buffer_index);
					exit_code = 1;
					break;
				}
				if (config_.webrtcEnabled() &&
					!webrtc_publisher.publish({codec_header.data(), codec_header.size(), 0, 0, true})) {
					std::cerr << "failed to push codec header to webrtc publisher\n";
					camera.requeueBuffer(frame.buffer_index);
					exit_code = 1;
					break;
				}
			} else {
				std::cerr << "warning: unable to fetch MPP codec header (SPS/PPS)\n";
			}

			rgb.resize(static_cast<std::size_t>(config_.model_width) * config_.model_height * 3);
			initialized = true;
		}

		rk3588::core::FramePacket latest;
		while (queue.pop_for(latest, std::chrono::milliseconds(0))) {
			camera.requeueBuffer(frame.buffer_index);
			frame = std::move(latest);
		}

		double preprocess_ms = 0.0;
		double infer_ms = 0.0;
		double fusion_ms = 0.0;
		double mlp_ms = 0.0;
		double track_ms = 0.0;
		double overlay_ms = 0.0;
		double encode_submit_ms = 0.0;

		const bool do_infer =
			(encode_frames % static_cast<std::uint64_t>(config_.infer_every_n_frames) == 0) ||
			last_detections.empty();
		if (do_infer) {
			const auto preprocess_begin = std::chrono::steady_clock::now();
			const bool rga_ok = rga.processDmaFdToRgbResize(frame.dma_fd, frame.pixel_format, rgb.data());
			preprocess_ms = std::chrono::duration<double, std::milli>(
				std::chrono::steady_clock::now() - preprocess_begin).count();
			if (!rga_ok) {
				std::cerr << "rga process failed frame_id=" << frame.frame_id << '\n';
				camera.requeueBuffer(frame.buffer_index);
				continue;
			}
			std::vector<YoloDetection> detections;
			const auto infer_begin = std::chrono::steady_clock::now();
			if (!rknn.inferRgb(rgb.data(),
							   rgb.size(),
							   static_cast<int>(frame.width),
							   static_cast<int>(frame.height),
							   &detections)) {
				infer_ms = std::chrono::duration<double, std::milli>(
					std::chrono::steady_clock::now() - infer_begin).count();
				std::cerr << "rknn infer failed frame_id=" << frame.frame_id << '\n';
				camera.requeueBuffer(frame.buffer_index);
				continue;
			}
			infer_ms = std::chrono::duration<double, std::milli>(
				std::chrono::steady_clock::now() - infer_begin).count();
			last_detections = std::move(detections);
		}

		rk3588::core::LidarMatchInfo lidar_match_info;
		std::uint64_t lidar_delta_ms = 0;
		rk3588::core::PointCloudPacket matched_cloud;
		const bool have_candidate_cloud = lidar_buffer.readClosestWithInfo(frame.timestamp_ms, matched_cloud, lidar_match_info);
		lidar_delta_ms = lidar_match_info.delta_ms;
		const std::uint64_t compensated_age_ms = lidar_match_info.estimated_scan_period_ms > 0
			? std::min(config_.lidar_max_age_ms,
					   std::max<std::uint64_t>(20U, lidar_match_info.estimated_scan_period_ms / 2 + 15U))
			: config_.lidar_max_age_ms;
		const bool has_lidar_cloud = have_candidate_cloud && lidar_delta_ms <= compensated_age_ms;

		const auto fusion_begin = std::chrono::steady_clock::now();
		distance_fusion.fuse(has_lidar_cloud ? &matched_cloud : nullptr,
							 &last_detections,
							 &fusion_diagnostics);
		fusion_ms = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - fusion_begin).count();

		if (mlp_cfg.enabled) {
			const auto mlp_begin = std::chrono::steady_clock::now();
			mlp_corrector.correct(&last_detections, &fusion_diagnostics);
			mlp_ms = std::chrono::duration<double, std::milli>(
				std::chrono::steady_clock::now() - mlp_begin).count();
		}

		std::vector<TrackObservation> track_observations;
		track_observations.reserve(last_detections.size());
		for (const auto& det : last_detections) {
			TrackObservation observation;
			observation.class_id = det.class_id;
			observation.confidence = det.confidence;
			observation.distance_m = det.distance_m;
			observation.left = det.left;
			observation.top = det.top;
			observation.right = det.right;
			observation.bottom = det.bottom;
			observation.angle_deg = fusion.pixelToCameraAngle(0.5F * static_cast<float>(det.left + det.right));
			track_observations.push_back(observation);
		}
		const auto track_begin = std::chrono::steady_clock::now();
		tracker.update(frame.frame_id, frame.timestamp_ms, track_observations, &track_estimates);
		for (std::size_t i = 0; i < last_detections.size() && i < track_estimates.size(); ++i) {
			if (track_estimates[i].filtered_distance_m >= 0.0F) {
				last_detections[i].distance_m = track_estimates[i].filtered_distance_m;
			}
		}
		track_ms = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - track_begin).count();

		if (pseudo_labels.enabled()) {
			pseudo_labels.writeFrame(config_.camera_device,
							 frame.frame_id,
							 frame.timestamp_ms,
							 has_lidar_cloud,
							 lidar_delta_ms,
							 config_.camera_fov_deg,
							 config_.lidar_offset_deg,
							 config_.lidar_fov_deg,
							 config_.lidar_window_half_deg,
							 config_.lidar_min_dist_m,
							 config_.lidar_max_dist_m,
							 config_.lidar_max_age_ms,
							 config_.calibration_profile_path,
							 last_detections,
							 track_estimates,
							 fusion_diagnostics);
		}

		bool ok = false;
		if (frame.pixel_format == V4L2_PIX_FMT_NV12 && frame.cpu_addr != 0) {
			const auto overlay_begin = std::chrono::steady_clock::now();
			if (config_.enable_video_overlay) {
				drawDetectionsNv12(reinterpret_cast<std::uint8_t*>(frame.cpu_addr),
						   static_cast<int>(frame.width),
						   static_cast<int>(frame.height),
						   static_cast<int>(frame.hor_stride > 0 ? frame.hor_stride : frame.width),
						   last_detections,
						   3);
			}

			const auto now = std::chrono::steady_clock::now();
			const double elapsed_sec =
				std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
			const std::string output_url = config_.webrtcEnabled() ? webrtc_publisher.rtcPlayUrl()
																   : streamer.publishUrl();
			if (config_.enable_video_overlay && config_.debug_video_hud) {
				const auto hud_lines = buildHudLines(frame.frame_id,
											 input_frames,
											 output_packets,
											 encode_frames,
											 config_.infer_every_n_frames,
											 do_infer,
											 last_detections,
											 lidar_scan_count.load(),
											 lidar_delta_ms,
											 frame.pixel_format,
											 actual_422_fourcc,
											 output_url.c_str(),
											 elapsed_sec);
				drawHudLinesNv12(reinterpret_cast<std::uint8_t*>(frame.cpu_addr),
							 static_cast<int>(frame.width),
							 static_cast<int>(frame.height),
							 static_cast<int>(frame.hor_stride > 0 ? frame.hor_stride : frame.width),
							 hud_lines,
							 8,
							 8,
							 2);
			}
			overlay_ms = std::chrono::duration<double, std::milli>(
				std::chrono::steady_clock::now() - overlay_begin).count();

			const auto encode_begin = std::chrono::steady_clock::now();
			ok = encoder.encodeFrame(frame, false);
			encode_submit_ms = std::chrono::duration<double, std::milli>(
				std::chrono::steady_clock::now() - encode_begin).count();
		} else if (isPacked422(frame.pixel_format) && frame.cpu_addr != 0) {
			const auto overlay_begin = std::chrono::steady_clock::now();
			if (actual_422_fourcc == 0) {
				if (config_.forced_422_fourcc != 0) {
					actual_422_fourcc = config_.forced_422_fourcc;
				} else if (frame.pixel_format == V4L2_PIX_FMT_YUYV) {
					actual_422_fourcc = V4L2_PIX_FMT_YUYV;
				} else {
					actual_422_fourcc = detectPacked422Fourcc(
						reinterpret_cast<const std::uint8_t*>(frame.cpu_addr),
						frame.width,
						frame.height,
						frame.hor_stride);
				}
				std::cout << "packed422 actual decode format=" << fourccName(actual_422_fourcc)
						  << " (camera says " << fourccName(frame.pixel_format) << ")\n";
			}

			const std::uint32_t nv12_stride = (frame.width + 15U) & ~15U;
			const std::size_t nv12_size = static_cast<std::size_t>(nv12_stride) * frame.height * 3U / 2U;
			if (nv12_scratch.size() != nv12_size) {
				nv12_scratch.resize(nv12_size);
			}

			bool converted = false;
			if (!config_.swap_uv) {
				converted = rga.processPacked422ToNv12(
					reinterpret_cast<const std::uint8_t*>(frame.cpu_addr),
					actual_422_fourcc,
					frame.hor_stride,
					nv12_scratch.data(),
					nv12_stride);
			}

			if (!converted) {
				if (!logged_rga_422_fallback && !config_.swap_uv) {
					std::cout << "packed422->nv12: fallback to CPU conversion"
							  << " fourcc=" << fourccName(actual_422_fourcc)
							  << " src_stride=" << frame.hor_stride
							  << " dst_stride=" << nv12_stride << '\n';
					logged_rga_422_fallback = true;
				}
				if (!convertPacked422ToNv12(reinterpret_cast<const std::uint8_t*>(frame.cpu_addr),
											frame.width,
											frame.height,
											frame.hor_stride,
											actual_422_fourcc,
											nv12_stride,
											config_.swap_uv,
											&nv12_scratch)) {
					std::cerr << "packed422->nv12 convert failed frame_id=" << frame.frame_id << '\n';
					camera.requeueBuffer(frame.buffer_index);
					continue;
				}
			}

			if (config_.enable_video_overlay) {
				drawDetectionsNv12(nv12_scratch.data(),
						   static_cast<int>(frame.width),
						   static_cast<int>(frame.height),
						   static_cast<int>(nv12_stride),
						   last_detections,
						   3);
			}

			const auto now = std::chrono::steady_clock::now();
			const double elapsed_sec =
				std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
			const std::string output_url = config_.webrtcEnabled() ? webrtc_publisher.rtcPlayUrl()
																   : streamer.publishUrl();
			if (config_.enable_video_overlay && config_.debug_video_hud) {
				const auto hud_lines = buildHudLines(frame.frame_id,
											 input_frames,
											 output_packets,
											 encode_frames,
											 config_.infer_every_n_frames,
											 do_infer,
											 last_detections,
											 lidar_scan_count.load(),
											 lidar_delta_ms,
											 frame.pixel_format,
											 actual_422_fourcc,
											 output_url.c_str(),
											 elapsed_sec);
				drawHudLinesNv12(nv12_scratch.data(),
							 static_cast<int>(frame.width),
							 static_cast<int>(frame.height),
							 static_cast<int>(nv12_stride),
							 hud_lines,
							 8,
							 8,
							 2);
			}
			overlay_ms = std::chrono::duration<double, std::milli>(
				std::chrono::steady_clock::now() - overlay_begin).count();

			const auto encode_begin = std::chrono::steady_clock::now();
			ok = encoder.encodeNv12Cpu(nv12_scratch.data(),
									   frame.width,
									   frame.height,
									   nv12_stride,
									   frame.pts_us,
									   frame.dts_us,
									   false);
				encode_submit_ms = std::chrono::duration<double, std::milli>(
					std::chrono::steady_clock::now() - encode_begin).count();
		} else {
			std::cerr << "unsupported pixel format or unavailable cpu_addr, fourcc=0x"
					  << std::hex << frame.pixel_format << std::dec
					  << " cpu_addr=" << frame.cpu_addr << '\n';
		}

		camera.requeueBuffer(frame.buffer_index);
		if (!ok) {
			std::cerr << "encode failed at frame_id=" << frame.frame_id << '\n';
			continue;
		}
		++input_frames;
		++encode_frames;

		while (true) {
			EncodedPacket pkt;
			const int got = encoder.getPacket(&pkt);
			if (got <= 0) {
				break;
			}
			const std::uint64_t dts_ms = pkt.dts_us >= 0 ? static_cast<std::uint64_t>(pkt.dts_us) / 1000U : 0;
			const std::uint64_t pts_ms = pkt.pts_us >= 0 ? static_cast<std::uint64_t>(pkt.pts_us) / 1000U : dts_ms;
			bool publish_ok = true;
			if (config_.rtspEnabled()) {
				publish_ok = streamer.pushPacket(pkt.data, pkt.len, dts_ms, pts_ms) && publish_ok;
			}
			if (config_.webrtcEnabled()) {
				publish_ok = webrtc_publisher.publish({pkt.data, pkt.len, dts_ms, pts_ms, false}) && publish_ok;
			}
			if (!publish_ok) {
				std::cerr << "stream push packet failed\n";
				encoder.releasePacket(&pkt);
				exit_code = 1;
				goto cleanup;
			}
			if (dump_file != nullptr) {
				(void)std::fwrite(pkt.data, 1, pkt.len, dump_file);
				std::fflush(dump_file);
			}
			++output_packets;
			encoder.releasePacket(&pkt);
		}

		if (telemetry.enabled()) {
			const auto now = std::chrono::steady_clock::now();
			const double elapsed_sec =
				std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
			std::uint32_t confirmed_target_count = 0;
			std::uint32_t valid_distance_target_count = 0;
			std::uint32_t ttc_alert_count = 0;
			for (std::size_t i = 0; i < last_detections.size(); ++i) {
				if (last_detections[i].distance_m >= 0.0F) {
					++valid_distance_target_count;
				}
				if (i < track_estimates.size() && track_estimates[i].confirmed) {
					++confirmed_target_count;
					if (track_estimates[i].ttc_s > 0.0F && track_estimates[i].ttc_s <= 3.0F) {
						++ttc_alert_count;
					}
				}
			}
			const std::string output_url = config_.webrtcEnabled() ? webrtc_publisher.rtcPlayUrl()
												   : streamer.publishUrl();
			RuntimeTelemetrySnapshot snapshot;
			snapshot.pipeline_name = "perception";
			snapshot.publish_mode = config_.publish_mode;
			snapshot.primary_url = output_url;
			snapshot.camera_device = config_.camera_device;
			snapshot.pixel_format = fourccName(actual_422_fourcc != 0 ? actual_422_fourcc : frame.pixel_format);
			snapshot.telemetry_ts_ms = nowSteadyMs();
			snapshot.frame_id = frame.frame_id;
			snapshot.input_frames = input_frames;
			snapshot.output_packets = output_packets;
			snapshot.encode_frames = encode_frames;
			snapshot.lidar_scan_count = lidar_scan_count.load();
			snapshot.lidar_delta_ms = lidar_delta_ms;
			snapshot.lidar_scan_period_ms = lidar_match_info.estimated_scan_period_ms;
			snapshot.lidar_allowed_age_ms = compensated_age_ms;
			snapshot.runtime_sec = elapsed_sec;
			snapshot.encode_fps = elapsed_sec > 1e-3 ? static_cast<double>(encode_frames) / elapsed_sec : 0.0;
			snapshot.capture_to_encode_ms =
				frame.timestamp_ms > 0 ? static_cast<double>(nowSteadyMs() - frame.timestamp_ms) : 0.0;
			snapshot.preprocess_ms = preprocess_ms;
			snapshot.infer_ms = infer_ms;
			snapshot.fusion_ms = fusion_ms;
			snapshot.track_ms = track_ms;
			snapshot.mlp_ms = mlp_ms;
			snapshot.overlay_ms = overlay_ms;
			snapshot.encode_submit_ms = encode_submit_ms;
			snapshot.lidar_points_total = has_lidar_cloud ? static_cast<std::uint32_t>(matched_cloud.points.size()) : 0U;
			snapshot.target_count = static_cast<std::uint32_t>(last_detections.size());
			snapshot.confirmed_target_count = confirmed_target_count;
			snapshot.valid_distance_target_count = valid_distance_target_count;
			snapshot.ttc_alert_count = ttc_alert_count;
			snapshot.center_angle_deg = wrapSignedAngleDeg(-config_.lidar_offset_deg);
			snapshot.camera_overlap_half_angle_deg = config_.camera_fov_deg * 0.5F;
			snapshot.did_infer = do_infer;
			snapshot.lidar_matched = has_lidar_cloud;
			snapshot.targets = buildTelemetryTargets(fusion, last_detections, &track_estimates, &fusion_diagnostics);
			snapshot.lidar_points = buildTelemetryLidarPoints(fusion, has_lidar_cloud ? &matched_cloud : nullptr);
			telemetry.maybeEmit(snapshot.telemetry_ts_ms, snapshot);
		}
	}

cleanup:
	camera.stop();
	queue.close();
	streamer.stop();
	webrtc_publisher.stop();
	stop_lidar();
	if (dump_file != nullptr) {
		std::fclose(dump_file);
		dump_file = nullptr;
	}

	std::cout << "mpp_encoder_demo done: input_frames=" << input_frames
			  << " output_packets=" << output_packets
			  << " encode_frames=" << encode_frames
			  << " run_seconds=" << config_.run_seconds << '\n';
	return exit_code;
}

}  // namespace rk3588::modules
