# RK3588 多模态感知系统

基于 RK3588（Orange Pi 5）的实时相机 + LiDAR 融合感知系统，支持目标检测、融合测距、多目标跟踪、视频推流与伪标签导出。

## 主要能力

- **硬件加速 Pipeline**：V4L2 DMA 采集 → RGA 前处理 → RKNN NPU 推理（YOLOv8n）→ MPP 硬编码 H.264，全链路零拷贝
- **多模态融合**：RPLIDAR A1 雷达与相机空间对齐，鲁棒聚类距离估计，为每个检测框注入距离信息
- **多目标跟踪**：IoU + 中心距离 + 角度 + 距离四维关联，支持短时遮挡 ghost 预测，输出 track_id / 速度 / TTC
- **RTSP / WebRTC 推流**：基于 ZLMediaKit，支持 NV12 叠加层（检测框 + HUD）
- **数据闭环**：按帧导出 JSONL 伪标签，可直接用于模型训练或后处理分析

## 快速开始

### 构建

```bash
cmake -S . -B build
cmake --build build -j
```

### 运行感知主程序

```bash
# 30 秒默认运行
./build/perception_app
```

```bash
# 启用伪标签 + 遥测导出
export RK3588_PSEUDO_LABEL_PATH=/tmp/rk3588_pseudo_labels.jsonl
export RK3588_TELEMETRY_PATH=/tmp/rk3588_telemetry.jsonl
./build/perception_app
```

## 架构

```
Camera (V4L2, YUYV 640x480)
  ├─→ RGA (YUYV→RGB 640x640) → RKNN (YOLOv8n) → detections
  │                                                    ↓
  │                           LiDAR → Ring Buffer → Fusion (distance)
  │                                                    ↓
  │                                         MultiTargetTracker (track_id, velocity, TTC)
  │                                                    ↓
  └─→ format convert (YUYV→NV12) → Overlay (draw boxes) → MPP (H.264) → RTSP/WebRTC
```

| 模块 | 位置 | 说明 |
|------|------|------|
| Camera | `src/camera/` | V4L2 采集，DMA buffer pool |
| RGA | `src/video/rga_processor.cpp` | 硬件加速 YUYV→RGB/NV12，DMA fd 零拷贝 |
| RKNN | `src/infer/rknn_runner.cpp` | YOLOv8n INT8 NPU 推理 |
| LiDAR | `src/lidar/` | RPLIDAR A1 驱动 + 环形缓冲区 |
| Fusion | `include/fusion/` | 角度窗映射 + 鲁棒聚类距离估计（header-only） |
| Tracker | `include/fusion/multi_target_tracker.hpp` | 多目标跟踪 + ghost 预测（header-only） |
| MPP | `src/video/mpp_encoder.cpp` | 硬件 H.264 编码 |
| Publish | `src/video/zlm_rtsp_publisher.cpp` | RTSP / WebRTC 推流 |
| Pipeline | `src/pipeline/perception_pipeline.cpp` | 主事件循环编排 |
| Overlay | `src/video/nv12_overlay.cpp` | NV12 帧上绘制检测框 + HUD |

核心设计决策：3 线程生产者-消费者模型、BoundedQueue 满时丢旧帧防积压、`infer_every_n_frames=5` 跳帧推理、fusion/tracker 全部 header-only。

## 测试

```bash
# 三场景测试（静态目标 / 接近目标 / 遮挡穿行），自动生成指标汇总
./scripts/run_phasec_suite.sh /dev/video0 640 480 75 reports/phasec

# 校验伪标签导出质量
python3 tools/diagnostics/validate_pseudo_labels.py --glob '/tmp/rk3588_pseudo_labels.jsonl*'

# 跟踪质量指标（retention / ID切换 / 碎片化）
python3 tools/diagnostics/analyze_tracking_metrics.py --glob '/tmp/rk3588_pseudo_labels.jsonl*'

# 自动验收判定
python3 tools/diagnostics/check_phasec_gate.py --run-dir reports/phasec/latest

# 长时间压力测试（30 分钟 × 3 轮）
./scripts/perf_stress_suite.sh 1800 3 reports/perf
```

## 关键环境变量

| 变量 | 作用 | 默认值 |
|------|------|--------|
| `RK3588_PSEUDO_LABEL_PATH` | 伪标签 JSONL 输出路径 | 空（不导出） |
| `RK3588_TELEMETRY_PATH` | 遥测 JSONL 输出路径 | 空（不导出） |
| `RK3588_DISTANCE_FUSION_MODE` | 融合模式：`robust` / `legacy` | `robust` |
| `RK3588_TRACKER_MIN_IOU` | 跟踪 IoU 阈值 | `0.05` |
| `RK3588_TRACKER_GHOST_KEEP_FRAMES` | ghost 保留帧数 | `4` |
| `RK3588_TRACKER_MAX_IDLE_FRAMES` | 轨迹最大空闲帧数 | `12` |
| `RK3588_DISABLE_VIDEO_OVERLAY` | 关闭检测框绘制 | 空（绘制） |
| `RK3588_DEBUG_VIDEO_HUD` | 开启调试 HUD | 空（关闭） |
| `RK3588_CALIBRATION_PROFILE` | 标定配置文件路径 | 空 |

## 文档

- [技术手册](docs/technical_manual.md) — 多线程架构、融合算法、跟踪算法详细设计

## 目录

```
.
├── apps/         应用入口（perception_app / stream_gateway）
├── src/          核心实现（camera / infer / lidar / video / pipeline）
├── include/      头文件（fusion 为 header-only）
├── tools/        标定、诊断、电机控制、WebRTC 调试
├── scripts/      测试与联调脚本
├── docs/         技术文档
├── models/       RKNN 模型文件 + 标签列表
├── cmake/        CMake 模块 + 第三方依赖查找器
├── demos/        独立 demo 程序
└── reports/      测试报告输出（gitignored）
```

## License

MIT
