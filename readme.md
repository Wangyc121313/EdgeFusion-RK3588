# RK3588 边缘多模态感知与自动预标注系统

本项目基于 RK3588 平台，构建相机 + LiDAR 的实时感知链路，支持目标检测、融合测距、时序跟踪、视频发布以及伪标签导出。

## 主要能力

- 实时感知链路：采集 -> 预处理 -> 推理 -> 融合 -> 跟踪 -> 编码/发布。
- 多模态融合：为检测目标注入距离信息，并保留融合诊断字段。
- 时序跟踪：输出稳定的目标 `track_id`、年龄、速度与 TTC 等信息。
- 数据闭环：支持按帧导出 JSONL 伪标签，可直接用于后处理或训练前清洗。

## 快速开始

### 1) 构建

```bash
cmake -S . -B build
cmake --build build -j
```

### 2) 启动感知主程序

```bash
./build/perception_app
```

### 3) 启用伪标签导出（推荐）

```bash
export RK3588_PSEUDO_LABEL_PATH=/tmp/rk3588_pseudo_labels.jsonl
export RK3588_PSEUDO_LABEL_MAX_LINES=5000
export RK3588_PSEUDO_LABEL_SEQUENCE_ID=exp_20260415_cam_lidar
export RK3588_TELEMETRY_PATH=/tmp/rk3588_telemetry.jsonl

./build/perception_app
```

也可以直接一键采集并校验：

```bash
./scripts/run_pseudo_label_capture.sh /dev/video0 640 480 60 /tmp/rk3588_pseudo_labels.jsonl /tmp/rk3588_telemetry.jsonl
```

### 4) 校验导出质量

```bash
python3 tools/diagnostics/validate_pseudo_labels.py --glob '/tmp/rk3588_pseudo_labels.jsonl*'
```

### 5) 评估跟踪质量（建议）

```bash
python3 tools/diagnostics/analyze_tracking_metrics.py --glob '/tmp/rk3588_pseudo_labels.jsonl*'
```

关键指标说明：

- `track_retention_ratio`：相邻帧中，已确认轨迹被持续保留的比例（越高越好）。
- `id_switch_proxy_rate`：基于相邻帧 IoU 匹配的 ID 切换代理指标（越低越好）。
- `track_fragmentation_rate`：同一轨迹被切成多个不连续片段的程度（越低越好）。

### 6) 三场景统一测试（推荐）

```bash
./scripts/run_phasec_suite.sh /dev/video0 640 480 75 reports/phasec
```

默认行为：

- 推流协议默认 `webrtc`。
- 默认启动 8090 Debug UI：`http://127.0.0.1:8090/?app=live&stream=camera`。

说明：

- 脚本会依次执行三类场景：`static_target`、`approaching_target`、`crossing_occlusion`。
- 每个场景会产出：`telemetry.jsonl`、`pseudo_labels.jsonl`、`video.h264`、`tracking_metrics.txt`、`config.env`。
- 测试完成后自动汇总核心指标并生成：
	- `metrics_table.md`
	- `metrics_table.csv`
	- `showcase/`（最小展示包：演示视频 + 指标表 + 配置说明）

若不希望每个场景手动按回车开始，可设置：

```bash
PHASEC_NO_PAUSE=1 ./scripts/run_phasec_suite.sh /dev/video0 640 480 75 reports/phasec
```

若需关闭 Debug UI，可设置：

```bash
START_DEBUG_UI=0 PHASEC_NO_PAUSE=1 ./scripts/run_phasec_suite.sh /dev/video0 640 480 75 reports/phasec
```

### 7) 自动验收判定（PASS/FAIL）

```bash
python3 tools/diagnostics/check_phasec_gate.py --run-dir reports/phasec/latest
```

说明：

- 默认检查三场景 `static_target`、`approaching_target`、`crossing_occlusion`。
- 满足阈值返回退出码 `0`（PASS），否则返回 `1`（FAIL）。
- 阈值可按现场要求调整，例如：

```bash
python3 tools/diagnostics/check_phasec_gate.py \
	--run-dir reports/phasec/latest \
	--retention-min 0.75 \
	--id-switch-max 0.15 \
	--fragmentation-max 0.50 \
	--latency-p95-max-ms 160
```

### 8) 指标专用脚本

三场景测试：

```bash
./scripts/run_resume_three_scenarios.sh /dev/video0 640 480 75 reports/phasec
```

产物：

- `reports/phasec/latest/resume_metrics_3scene.md`
- `reports/phasec/latest/resume_metrics_3scene.json`

整体压测：

```bash
# 半小时压力测试
PERF_DURATION_S=1800 PERF_ROUNDS=1 ./scripts/run_resume_overall_test.sh /dev/video0 640 480
```

说明：

- 默认 `RUN_THREE_SCENES=0`，只跑整体压测。
- 若 `reports/phasec/latest/resume_metrics_3scene.json` 存在，会自动复用历史三场景指标写入总摘要。
- 若不存在历史三场景结果，则总摘要中三场景部分标记为 `SKIPPED`。
- 如需在本次总测中一并重跑三场景测试，可显式启用：

```bash
RUN_THREE_SCENES=1 PHASE_SECONDS=75 PERF_DURATION_S=1800 PERF_ROUNDS=1 ./scripts/run_resume_overall_test.sh /dev/video0 640 480
```

产物：

- `reports/resume/latest/resume_overall_summary.md`
- `reports/resume/latest/resume_overall_summary.json`

## 关键环境变量

- `RK3588_PSEUDO_LABEL_PATH`：伪标签输出路径。设为 `-` 时输出到标准输出。
- `RK3588_PSEUDO_LABEL_MAX_LINES`：单个伪标签文件最大行数，超过后自动滚动到 `.1`、`.2`。
- `RK3588_PSEUDO_LABEL_SEQUENCE_ID`：伪标签序列编号，用于区分不同采集批次。
- `RK3588_TELEMETRY_PATH`：运行遥测 JSONL 输出路径。
- `RK3588_TELEMETRY_INTERVAL_MS`：遥测输出间隔（毫秒）。
- `RK3588_DISTANCE_FUSION_MODE`：距离融合模式，`robust`（默认）或 `legacy`。
- `RK3588_TRACKER_MIN_IOU`：跟踪匹配最小 IoU 阈值（建议 0.03~0.15）。
- `RK3588_TRACKER_IOU_WEIGHT`：IoU 在匹配代价中的权重（建议 0.2~0.6）。
- `RK3588_TRACKER_GHOST_KEEP_FRAMES`：短时丢检保留帧数（建议 3~6）。
- `RK3588_TRACKER_MAX_IDLE_FRAMES`：轨迹最大空闲帧数（建议 >= ghost_keep）。
- `RK3588_TRACKER_CENTER_VEL_ALPHA`：中心速度平滑系数（越大越敏捷）。
- `RK3588_TRACKER_GHOST_DECAY`：ghost 外推速度衰减系数（越小越保守）。
- `RK3588_DISABLE_VIDEO_OVERLAY`：设为非 `0` 值时关闭视频叠加绘制。
- `RK3588_DEBUG_VIDEO_HUD`：设为非 `0` 值时开启调试 HUD。

## 伪标签格式（每帧）

- 顶层字段：`schema`、`sequence_id`、`source_fps`、`camera_device`、`frame_id`、`timestamp_ms`、`lidar_matched`、`lidar_delta_ms`、`sensor_snapshot`。
- `sensor_snapshot`：`camera_fov_deg`、`lidar_offset_deg`、`lidar_fov_deg`、`lidar_window_half_deg`、`lidar_min_dist_m`、`lidar_max_dist_m`、`lidar_max_age_ms`、`calibration_profile`。
- 目标字段：`class_id`、`class_name`、`confidence`、`distance_m`、`bbox`、`track_id`、`track_age_frames`、`track_idle_frames`、`track_confirmed`、`track_is_ghost`、`track_angle_deg`。

## 目录概览

- `apps/`：应用入口（`perception_main.cpp`、`stream_gateway_main.cpp`）。
- `src/`：核心实现（相机、推理、融合、跟踪、视频、pipeline）。
- `include/`：对外头文件。
- `tools/`：诊断、标定、电机控制与调试工具。
- `scripts/`：联调脚本与压力测试脚本。
- `docs/`：架构、构建、运行、标定等详细文档。

## 架构设计入口

- `docs/ARCHITECTURE_INTERFACE_GUIDE.md`：系统分层、接口定义、数据流向图、前后端职责切分。
- `docs/control_plane_openapi_v1.yaml`：控制面后端接口草案（OpenAPI 3.0）。