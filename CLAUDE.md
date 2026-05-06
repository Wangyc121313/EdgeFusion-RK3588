# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Test / Validate

```bash
# Quick: build + run perception app (30s default)
./build/perception_app

# PhaseC 3-scenario suite (tracking + fusion evaluation)
./scripts/run_phasec_suite.sh /dev/video0 640 480 75 reports/phasec

# Validate exported pseudo-labels
python3 tools/diagnostics/validate_pseudo_labels.py --glob '/tmp/rk3588_pseudo_labels.jsonl*'

# Tracking quality metrics
python3 tools/diagnostics/analyze_tracking_metrics.py --glob '/tmp/rk3588_pseudo_labels.jsonl*'

# Automated PASS/FAIL gating
python3 tools/diagnostics/check_phasec_gate.py --run-dir reports/phasec/latest

# Stress test (duration_s rounds)
./scripts/perf_stress_suite.sh 1800 3 reports/perf
```

Single demo binaries live in `build/` — e.g. `./build/camera_v4l2_demo`, `./build/rplidar_demo`, `./build/rga_demo`, `./build/pipeline_thread_demo`.

## Architecture

Target: **Orange Pi 5 (RK3588, ARM64 Ubuntu 22.04)**. C++17, CMake build.

### Namespaces

- `rk3588::core` — data types (`FramePacket`, `PointCloudPacket`), `BoundedQueue<T>`, `LidarRingBuffer`
- `rk3588::modules` — all domain modules and pipeline orchestration

### Key module layers (bottom-up)

| Layer | Source | Purpose |
|-------|--------|---------|
| Camera | `src/camera/` | V4L2 capture, DMA buffer pool, produces `FramePacket` into bounded queue |
| RGA | `src/video/rga_processor.cpp` | HW-accelerated YUYV→NV12, NV12→RGB resize; DMA fd in/out, zero CPU copy |
| RKNN | `src/infer/rknn_runner.cpp` | YOLOv8n INT8 inference on NPU (6 TOPS); receives RGB buffer from RGA |
| LiDAR | `src/lidar/` | RPLIDAR A1 via SDK; `LidarReader` reads scans, `LidarAdapter` normalizes into `PointCloudPacket`, written into `LidarRingBuffer` |
| Fusion | `include/fusion/sensor_fusion.hpp`, `detection_distance_fusion.hpp`, `multi_target_tracker.hpp` | Header-only. Spatial angle-zone mapping (LiDAR→camera angles), robust clustering distance estimation, multi-target IoU+center tracking with ghost prediction |
| MPP Encoder | `src/video/mpp_encoder.cpp` | RK3588 HW H.264 encoder (NV12 input) |
| Publish | `src/video/zlm_rtsp_publisher.cpp`, `webrtc_publisher.cpp`, `publisher_hub.cpp` | ZLMediaKit RTSP / WebRTC streaming |
| Pipeline | `src/pipeline/perception_pipeline.cpp` | Main orchestration — wires camera→RGA→RKNN + LiDAR→fusion→tracking→overlay→encode→publish in a single event loop |
| Overlay | `src/video/nv12_overlay.cpp` | Draws detection boxes + optional debug HUD directly on NV12 frames |

### Data flow (perception pipeline)

```
Camera (V4L2, YUYV 640x480)
  ├─→ RGA (YUYV→RGB 640x640) → RKNN (YOLOv8n) → detections
  │                                                    ↓
  │                           LiDAR → ring buffer → Fusion (distance)
  │                                                    ↓
  │                                          MultiTargetTracker (track_id, velocity, TTC)
  │                                                    ↓
  └─→ format convert (YUYV→NV12) → Overlay (draw boxes) → MPP (H.264) → RTSP/WebRTC
```

Key design decisions:
- **DMA zero-copy**: Camera DMA fd → RGA → RKNN input tensor, no `memcpy` into CPU buffers. `BoundedQueue` carries lightweight `FramePacket` (index + fd, not pixel data).
- **Inference skip**: `infer_every_n_frames` (default 5) — RKNN runs on subset of frames; detection results are reused on intermediate frames.
- **Bounded queue with drop**: When full, oldest frame is discarded to bound latency.
- **Header-only fusion/tracking**: `sensor_fusion.hpp`, `detection_distance_fusion.hpp`, `multi_target_tracker.hpp` are all header-only (no corresponding `.cpp`).

### Three app entrypoints

- `apps/perception_main.cpp` → `perception_app` — full perception pipeline (camera + infer + lidar + tracking + stream)
- `apps/rtsp_pipeline_main.cpp` → `camera_rga_rknn_rtsp_demo` — simplified RTSP-only pipeline
- `apps/stream_gateway_main.cpp` → `stream_gateway_app` — stream gateway

### Configuration

`AppConfig` (in `include/pipeline/app_config.hpp`) holds all tunables. CLI args (positional, 23 params) set base values; env vars override for targeted tuning. Key env vars:

| Variable | Purpose |
|----------|---------|
| `RK3588_PSEUDO_LABEL_PATH` | JSONL pseudo-label output path |
| `RK3588_TELEMETRY_PATH` | Runtime telemetry JSONL path |
| `RK3588_DISTANCE_FUSION_MODE` | `robust` (default) or `legacy` |
| `RK3588_TRACKER_*` | Tracker tuning (min_iou, iou_weight, ghost_keep, max_idle, etc.) |
| `RK3588_CALIBRATION_PROFILE` | Path to calibration profile JSON |
| `RK3588_DISABLE_VIDEO_OVERLAY` | Set to non-0 to skip drawing boxes |
| `RK3588_DEBUG_VIDEO_HUD` | Set to non-0 for on-screen debug overlay |

### Third-party dependencies (cmake/deps/)

- **MPP** — Rockchip Media Process Platform (HW H.264 encode)
- **RGA** — Rockchip Raster Graphic Acceleration (HW resize/CSC)
- **RKNN** — Rockchip NPU runtime (YOLOv8n inference)
- **RPLIDAR SDK** — Slamtec RPLIDAR A1 driver
- **ZLMediaKit** — RTSP/WebRTC streaming server (C API)
- **FFmpeg** — used by ZLMediaKit/WebRTC internally

### Directory conventions

- `include/<module>/` — public headers, structured by domain
- `include/rk3588/<module>/` — alternate header namespace mirrors (symlinks or copies)
- `src/<module>/` — corresponding `.cpp` implementations
- `cmake/modules/` — one `.cmake` per library target
- `cmake/deps/` — third-party dependency finders
- `demos/` — standalone demo programs (camera, encode, fusion, lidar, video)
- `tools/` — calibration, diagnostics (Python), motor control, WebRTC debug UI
- `scripts/` — shell scripts for testing, profiling, and device setup
- `reports/` — output directory for perf runs and PhaseC results (gitignored)
- `models/` — RKNN model file and label list
