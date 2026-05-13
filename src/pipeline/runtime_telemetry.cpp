#include "pipeline/runtime_telemetry.hpp"

#include <cstdio>
#include <sstream>
#include <utility>

namespace rk3588::modules {

RuntimeTelemetrySink::RuntimeTelemetrySink(std::string output_path, std::uint32_t interval_ms)
    : output_path_(std::move(output_path)), interval_ms_(interval_ms == 0 ? 1000 : interval_ms) {
    use_stdout_ = output_path_ == "-";
}

RuntimeTelemetrySink::~RuntimeTelemetrySink() {
    if (handle_ != nullptr && !use_stdout_) {
        std::fclose(static_cast<FILE*>(handle_));
        handle_ = nullptr;
    }
}

bool RuntimeTelemetrySink::enabled() const {
    return !output_path_.empty();
}

void RuntimeTelemetrySink::maybeEmit(std::uint64_t now_ms, const RuntimeTelemetrySnapshot& snapshot) {
    if (!enabled()) {
        return;
    }
    if (last_emit_ms_ != 0 && now_ms > last_emit_ms_ && now_ms - last_emit_ms_ < interval_ms_) {
        return;
    }
    if (!ensureOpen()) {
        return;
    }
    last_emit_ms_ = now_ms;
    writeSnapshot(snapshot);
}

bool RuntimeTelemetrySink::ensureOpen() {
    if (handle_ != nullptr) {
        return true;
    }
    if (!enabled()) {
        return false;
    }
    if (use_stdout_) {
        handle_ = stdout;
        return true;
    }

    handle_ = std::fopen(output_path_.c_str(), "a");
    return handle_ != nullptr;
}

std::string RuntimeTelemetrySink::escapeJson(const std::string& text) {
    std::string out;
    out.reserve(text.size() + 8);
    for (char ch : text) {
        switch (ch) {
            case '\\':
                out += "\\\\";
                break;
            case '"':
                out += "\\\"";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += ch;
                break;
        }
    }
    return out;
}

void RuntimeTelemetrySink::writeSnapshot(const RuntimeTelemetrySnapshot& snapshot) {
    auto* file = static_cast<FILE*>(handle_);
    if (file == nullptr) {
        return;
    }

    std::ostringstream json;
    json << '{'
         << "\"pipeline\":\"" << escapeJson(snapshot.pipeline_name) << "\"," 
         << "\"publish_mode\":\"" << escapeJson(snapshot.publish_mode) << "\"," 
         << "\"primary_url\":\"" << escapeJson(snapshot.primary_url) << "\"," 
         << "\"camera_device\":\"" << escapeJson(snapshot.camera_device) << "\"," 
         << "\"pixel_format\":\"" << escapeJson(snapshot.pixel_format) << "\"," 
         << "\"ts_ms\":" << snapshot.telemetry_ts_ms << ','
         << "\"runtime_sec\":" << snapshot.runtime_sec << ','
         << "\"frame_id\":" << snapshot.frame_id << ','
         << "\"input_frames\":" << snapshot.input_frames << ','
         << "\"output_packets\":" << snapshot.output_packets << ','
         << "\"encode_frames\":" << snapshot.encode_frames << ','
         << "\"encode_fps\":" << snapshot.encode_fps << ','
         << "\"center_angle_deg\":" << snapshot.center_angle_deg << ','
         << "\"camera_overlap_half_angle_deg\":" << snapshot.camera_overlap_half_angle_deg << ','
         << "\"did_infer\":" << (snapshot.did_infer ? "true" : "false") << ','
         << "\"lidar_matched\":" << (snapshot.lidar_matched ? "true" : "false") << ','
         << "\"lidar_scan_count\":" << snapshot.lidar_scan_count << ','
         << "\"lidar_delta_ms\":" << snapshot.lidar_delta_ms << ','
         << "\"lidar_scan_period_ms\":" << snapshot.lidar_scan_period_ms << ','
         << "\"lidar_allowed_age_ms\":" << snapshot.lidar_allowed_age_ms << ','
         << "\"capture_to_encode_ms\":" << snapshot.capture_to_encode_ms << ','
         << "\"preprocess_ms\":" << snapshot.preprocess_ms << ','
         << "\"infer_ms\":" << snapshot.infer_ms << ','
         << "\"fusion_ms\":" << snapshot.fusion_ms << ','
         << "\"track_ms\":" << snapshot.track_ms << ','
         << "\"mlp_ms\":" << snapshot.mlp_ms << ','
         << "\"overlay_ms\":" << snapshot.overlay_ms << ','
         << "\"encode_submit_ms\":" << snapshot.encode_submit_ms << ','
         << "\"lidar_points_total\":" << snapshot.lidar_points_total << ','
         << "\"target_count\":" << snapshot.target_count << ','
         << "\"confirmed_target_count\":" << snapshot.confirmed_target_count << ','
         << "\"valid_distance_target_count\":" << snapshot.valid_distance_target_count << ','
         << "\"ttc_alert_count\":" << snapshot.ttc_alert_count << ','
         << "\"targets\":[";

    for (std::size_t i = 0; i < snapshot.targets.size(); ++i) {
        const auto& target = snapshot.targets[i];
        if (i != 0) {
            json << ',';
        }
        json << '{'
                         << "\"track_id\":" << target.track_id << ','
                         << "\"track_age_frames\":" << target.track_age_frames << ','
                         << "\"track_idle_frames\":" << target.track_idle_frames << ','
                         << "\"track_confirmed\":" << (target.track_confirmed ? "true" : "false") << ','
                         << "\"track_is_ghost\":" << (target.track_is_ghost ? "true" : "false") << ','
             << "\"class_id\":" << target.class_id << ','
             << "\"class_name\":\"" << escapeJson(target.class_name) << "\"," 
             << "\"confidence\":" << target.confidence << ','
               << "\"angle_deg\":" << target.angle_deg << ','
             << "\"distance_m\":" << target.distance_m << ','
                             << "\"lateral_offset_m\":" << target.lateral_offset_m << ','
                             << "\"radial_velocity_mps\":" << target.radial_velocity_mps << ','
                             << "\"lateral_velocity_mps\":" << target.lateral_velocity_mps << ','
                             << "\"closing_speed_mps\":" << target.closing_speed_mps << ','
                             << "\"ttc_s\":" << target.ttc_s << ','
               << "\"raw_distance_m\":" << target.raw_distance_m << ','
               << "\"support_points\":" << target.support_points << ','
               << "\"cluster_points\":" << target.cluster_points << ','
               << "\"cluster_score\":" << target.cluster_score << ','
               << "\"used_fallback\":" << (target.used_fallback ? "true" : "false") << ','
               << "\"used_temporal_smoothing\":" << (target.used_temporal_smoothing ? "true" : "false") << ','
             << "\"rejected_by_sanity\":" << (target.rejected_by_sanity ? "true" : "false") << ','
             << "\"left\":" << target.left << ','
             << "\"top\":" << target.top << ','
             << "\"right\":" << target.right << ','
             << "\"bottom\":" << target.bottom
             << '}';
    }

    json << "],\"lidar_points\":[";

    for (std::size_t i = 0; i < snapshot.lidar_points.size(); ++i) {
        const auto& point = snapshot.lidar_points[i];
        if (i != 0) {
            json << ',';
        }
        json << '{'
             << "\"angle_deg\":" << point.angle_deg << ','
             << "\"distance_m\":" << point.distance_m << ','
             << "\"x_m\":" << point.x_m << ','
             << "\"z_m\":" << point.z_m
             << '}';
    }

    json << "]}\n";
    const std::string text = json.str();
    (void)std::fwrite(text.data(), 1, text.size(), file);
    std::fflush(file);
}

}  // namespace rk3588::modules