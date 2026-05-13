#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rk3588::modules {

struct TelemetryLidarPoint {
    float angle_deg = 0.0F;
    float distance_m = 0.0F;
    float x_m = 0.0F;
    float z_m = 0.0F;
};

struct TelemetryTarget {
    int track_id = -1;
    std::uint32_t track_age_frames = 0;
    std::uint32_t track_idle_frames = 0;
    bool track_confirmed = false;
    bool track_is_ghost = false;
    int class_id = -1;
    std::string class_name;
    float confidence = 0.0F;
    float angle_deg = 0.0F;
    float distance_m = -1.0F;
    float lateral_offset_m = 0.0F;
    float radial_velocity_mps = 0.0F;
    float lateral_velocity_mps = 0.0F;
    float closing_speed_mps = 0.0F;
    float ttc_s = -1.0F;
    float raw_distance_m = -1.0F;
    int support_points = 0;
    int cluster_points = 0;
    float cluster_score = 0.0F;
    bool used_fallback = false;
    bool used_temporal_smoothing = false;
    bool rejected_by_sanity = false;
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
};

struct RuntimeTelemetrySnapshot {
    std::string pipeline_name;
    std::string publish_mode;
    std::string primary_url;
    std::string camera_device;
    std::string pixel_format;
    std::uint64_t telemetry_ts_ms = 0;
    std::uint64_t frame_id = 0;
    std::uint64_t input_frames = 0;
    std::uint64_t output_packets = 0;
    std::uint64_t encode_frames = 0;
    std::uint64_t lidar_scan_count = 0;
    std::uint64_t lidar_delta_ms = 0;
    std::uint64_t lidar_scan_period_ms = 0;
    std::uint64_t lidar_allowed_age_ms = 0;
    double runtime_sec = 0.0;
    double encode_fps = 0.0;
    double capture_to_encode_ms = 0.0;
    double preprocess_ms = 0.0;
    double infer_ms = 0.0;
    double fusion_ms = 0.0;
    double track_ms = 0.0;
    double mlp_ms = 0.0;
    double overlay_ms = 0.0;
    double encode_submit_ms = 0.0;
    std::uint32_t lidar_points_total = 0;
    std::uint32_t target_count = 0;
    std::uint32_t confirmed_target_count = 0;
    std::uint32_t valid_distance_target_count = 0;
    std::uint32_t ttc_alert_count = 0;
    float center_angle_deg = 0.0F;
    float camera_overlap_half_angle_deg = 30.0F;
    bool did_infer = false;
    bool lidar_matched = false;
    std::vector<TelemetryTarget> targets;
    std::vector<TelemetryLidarPoint> lidar_points;
};

class RuntimeTelemetrySink {
public:
    RuntimeTelemetrySink(std::string output_path, std::uint32_t interval_ms);
    ~RuntimeTelemetrySink();

    RuntimeTelemetrySink(const RuntimeTelemetrySink&) = delete;
    RuntimeTelemetrySink& operator=(const RuntimeTelemetrySink&) = delete;

    [[nodiscard]] bool enabled() const;
    void maybeEmit(std::uint64_t now_ms, const RuntimeTelemetrySnapshot& snapshot);

private:
    bool ensureOpen();
    static std::string escapeJson(const std::string& text);
    void writeSnapshot(const RuntimeTelemetrySnapshot& snapshot);

private:
    std::string output_path_;
    std::uint32_t interval_ms_ = 1000;
    std::uint64_t last_emit_ms_ = 0;
    void* handle_ = nullptr;
    bool use_stdout_ = false;
};

}  // namespace rk3588::modules