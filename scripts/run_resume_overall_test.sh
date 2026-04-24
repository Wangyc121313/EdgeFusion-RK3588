#!/usr/bin/env bash
set -euo pipefail

# Run resume-oriented overall benchmark:
# 1) Overall stress/performance rounds (throughput + resource metrics)
# 2) Optionally run three standard scenes (quality + latency metrics)
# 3) Merge to one resume summary file
#
# Usage:
#   scripts/run_resume_overall_test.sh [camera_device] [width] [height]
# Example:
#   scripts/run_resume_overall_test.sh /dev/video0 640 480

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

CAMERA_DEVICE="${1:-/dev/video0}"
CAMERA_WIDTH="${2:-640}"
CAMERA_HEIGHT="${3:-480}"

PHASE_SECONDS="${PHASE_SECONDS:-75}"
PERF_DURATION_S="${PERF_DURATION_S:-300}"
PERF_ROUNDS="${PERF_ROUNDS:-2}"
PHASE_OUT_ROOT="${PHASE_OUT_ROOT:-reports/phasec}"
PERF_OUT_ROOT="${PERF_OUT_ROOT:-reports/perf}"
RESUME_OUT_ROOT="${RESUME_OUT_ROOT:-reports/resume}"
RUN_THREE_SCENES="${RUN_THREE_SCENES:-0}"

mkdir -p "$RESUME_OUT_ROOT"
RUN_TS="$(date +%Y%m%d_%H%M%S)"
OVERALL_DIR="$RESUME_OUT_ROOT/run_$RUN_TS"
mkdir -p "$OVERALL_DIR"
ln -sfn "run_$RUN_TS" "$RESUME_OUT_ROOT/latest"

echo "[resume_overall] start"
echo "[resume_overall] output=$OVERALL_DIR"
echo "[resume_overall] phase_seconds=$PHASE_SECONDS perf_duration_s=$PERF_DURATION_S perf_rounds=$PERF_ROUNDS run_three_scenes=$RUN_THREE_SCENES"

PHASE_RUN_DIR=""
PHASE_MD=""
PHASE_JSON=""
PHASE_SOURCE_MODE="skipped"

case "${RUN_THREE_SCENES,,}" in
    1|true|yes|on)
        echo "[resume_overall] running three-scene suite before perf"
        scripts/run_resume_three_scenarios.sh \
            "$CAMERA_DEVICE" \
            "$CAMERA_WIDTH" \
            "$CAMERA_HEIGHT" \
            "$PHASE_SECONDS" \
            "$PHASE_OUT_ROOT"

        PHASE_RUN_DIR="$(readlink -f "$PHASE_OUT_ROOT/latest")"
        PHASE_MD="$PHASE_RUN_DIR/resume_metrics_3scene.md"
        PHASE_JSON="$PHASE_RUN_DIR/resume_metrics_3scene.json"
        if [[ ! -f "$PHASE_JSON" ]]; then
            echo "error: missing phase summary json: $PHASE_JSON"
            exit 2
        fi
        PHASE_SOURCE_MODE="fresh"
        ;;
    *)
        if [[ -L "$PHASE_OUT_ROOT/latest" ]]; then
            CANDIDATE_PHASE_RUN_DIR="$(readlink -f "$PHASE_OUT_ROOT/latest" || true)"
            if [[ -n "${CANDIDATE_PHASE_RUN_DIR:-}" && -f "$CANDIDATE_PHASE_RUN_DIR/resume_metrics_3scene.json" ]]; then
                PHASE_RUN_DIR="$CANDIDATE_PHASE_RUN_DIR"
                PHASE_MD="$PHASE_RUN_DIR/resume_metrics_3scene.md"
                PHASE_JSON="$PHASE_RUN_DIR/resume_metrics_3scene.json"
                PHASE_SOURCE_MODE="reuse_latest"
                echo "[resume_overall] reuse latest three-scene summary: $PHASE_RUN_DIR"
            fi
        fi

        if [[ -z "$PHASE_JSON" ]]; then
            PHASE_RUN_DIR="N/A (skipped)"
            PHASE_JSON="$OVERALL_DIR/_phase_placeholder.json"
            cat > "$PHASE_JSON" <<'JSON'
{
    "gate_status": "SKIPPED",
    "total_frames": 0,
    "avg_track_retention_ratio": null,
    "worst_id_switch_proxy_rate": null,
    "worst_track_fragmentation_rate": null,
    "worst_latency_p95_ms": null
}
JSON
            PHASE_SOURCE_MODE="skipped"
            echo "[resume_overall] three-scene suite skipped (RUN_THREE_SCENES=$RUN_THREE_SCENES)"
        fi
        ;;
esac

scripts/perf_stress_suite.sh "$PERF_DURATION_S" "$PERF_ROUNDS" "$PERF_OUT_ROOT"
PERF_RUN_DIR="$(readlink -f "$PERF_OUT_ROOT/latest")"
PERF_CSV="$PERF_RUN_DIR/overall_summary.csv"
PERF_REPORT="$PERF_RUN_DIR/report.md"
if [[ ! -f "$PERF_CSV" ]]; then
  echo "error: missing perf overall summary: $PERF_CSV"
  exit 2
fi

OVERALL_MD="$OVERALL_DIR/resume_overall_summary.md"
OVERALL_JSON="$OVERALL_DIR/resume_overall_summary.json"

python3 - "$PHASE_JSON" "$PERF_CSV" "$OVERALL_MD" "$OVERALL_JSON" "$PHASE_RUN_DIR" "$PERF_RUN_DIR" "$PHASE_SOURCE_MODE" <<'PY'
import csv
import json
import math
import statistics
import sys

phase_json, perf_csv, md_path, json_path, phase_run_dir, perf_run_dir, phase_source_mode = sys.argv[1:8]

with open(phase_json, "r", encoding="utf-8") as f:
    phase = json.load(f)

rows = []
with open(perf_csv, "r", encoding="utf-8", newline="") as f:
    rows = list(csv.DictReader(f))

if not rows:
    raise SystemExit("no rows in perf overall summary")

def vals(key):
    out = []
    for r in rows:
        try:
            out.append(float(r.get(key, "nan")))
        except ValueError:
            out.append(float("nan"))
    return [x for x in out if not math.isnan(x)]

def m(key):
    v = vals(key)
    return statistics.fmean(v) if v else float("nan")

def mx(key):
    v = vals(key)
    return max(v) if v else float("nan")

def mn(key):
    v = vals(key)
    return min(v) if v else float("nan")

def fmt(x, d=3):
    if isinstance(x, float) and math.isnan(x):
        return "nan"
    if isinstance(x, float):
        return f"{x:.{d}f}"
    return str(x)

exit_codes = vals("exit_code")
round_count = len(rows)
success_rounds = sum(1 for x in exit_codes if x == 0.0)

overall = {
    "phase_run_dir": phase_run_dir,
    "perf_run_dir": perf_run_dir,
    "phase_source_mode": phase_source_mode,
    "scene_gate_status": phase.get("gate_status", "UNKNOWN"),
    "scene_total_frames": phase.get("total_frames", 0),
    "scene_avg_retention": phase.get("avg_track_retention_ratio"),
    "scene_worst_id_switch": phase.get("worst_id_switch_proxy_rate"),
    "scene_worst_fragmentation": phase.get("worst_track_fragmentation_rate"),
    "scene_worst_latency_p95_ms": phase.get("worst_latency_p95_ms"),
    "perf_round_count": round_count,
    "perf_success_rounds": success_rounds,
    "perf_success_ratio": (success_rounds / round_count) if round_count > 0 else float("nan"),
    "perf_avg_fps": m("avg_fps"),
    "perf_avg_fps_util_pct": m("fps_util_pct"),
    "perf_peak_cpu_total_pct": mx("cpu_total_pct_mean"),
    "perf_peak_cpu_proc_pct": mx("cpu_proc_pct_mean"),
    "perf_peak_rss_mb": mx("rss_mb_mean"),
    "perf_peak_temp_cpu_c": mx("temp_cpu_c_mean"),
    "perf_avg_npu_pct": m("rknpu_pct_mean"),
    "perf_avg_encode_fps": m("encode_fps_mean"),
    "perf_avg_e2e_ms": m("capture_to_encode_ms_mean"),
    "perf_avg_infer_ms": m("infer_ms_mean"),
    "perf_avg_lidar_match_pct": m("lidar_match_pct_mean"),
    "perf_avg_lidar_delta_ms": m("lidar_delta_ms_mean"),
}

with open(json_path, "w", encoding="utf-8") as f:
    json.dump(overall, f, ensure_ascii=False, indent=2)

with open(md_path, "w", encoding="utf-8") as f:
    f.write("# 简历指标总摘要（场景质量 + 系统性能）\n\n")
    f.write("## 一、三场景质量指标\n\n")
    if phase_source_mode == "fresh":
        f.write("- 数据来源: 本次总测中重新执行三场景。\n")
    elif phase_source_mode == "reuse_latest":
        f.write("- 数据来源: 复用历史 latest 三场景结果（本次未重跑三场景）。\n")
    else:
        f.write("- 数据来源: 本次总测跳过三场景（仅整体压测）。\n")
    f.write(f"- Gate 验收: **{overall['scene_gate_status']}**\n")
    f.write(f"- 总样本帧数: {overall['scene_total_frames']}\n")
    f.write(f"- Track 保持率均值: {fmt(overall['scene_avg_retention'])}\n")
    f.write(f"- ID 切换代理率最差: {fmt(overall['scene_worst_id_switch'])}\n")
    f.write(f"- 轨迹碎片率最差: {fmt(overall['scene_worst_fragmentation'])}\n")
    f.write(f"- 端到端时延P95最差(ms): {fmt(overall['scene_worst_latency_p95_ms'])}\n\n")

    f.write("## 二、整体性能指标\n\n")
    f.write(f"- 压测轮次通过率: {overall['perf_success_rounds']}/{overall['perf_round_count']} ({fmt(overall['perf_success_ratio']*100 if not math.isnan(overall['perf_success_ratio']) else float('nan'), 2)}%)\n")
    f.write(f"- 平均输出帧率(FPS): {fmt(overall['perf_avg_fps'], 2)}\n")
    f.write(f"- 平均帧率利用率(%): {fmt(overall['perf_avg_fps_util_pct'], 2)}\n")
    f.write(f"- 峰值系统CPU占用(%): {fmt(overall['perf_peak_cpu_total_pct'], 2)}\n")
    f.write(f"- 峰值进程CPU占用(%): {fmt(overall['perf_peak_cpu_proc_pct'], 2)}\n")
    f.write(f"- 峰值RSS内存(MB): {fmt(overall['perf_peak_rss_mb'], 2)}\n")
    f.write(f"- 峰值CPU温度(℃): {fmt(overall['perf_peak_temp_cpu_c'], 2)}\n")
    f.write(f"- 平均NPU负载(%): {fmt(overall['perf_avg_npu_pct'], 2)}\n")
    f.write(f"- 平均编码FPS: {fmt(overall['perf_avg_encode_fps'], 2)}\n")
    f.write(f"- 平均端到端时延(ms): {fmt(overall['perf_avg_e2e_ms'], 2)}\n")
    f.write(f"- 平均单帧推理耗时(ms): {fmt(overall['perf_avg_infer_ms'], 2)}\n")
    f.write(f"- 平均LiDAR匹配率(%): {fmt(overall['perf_avg_lidar_match_pct'], 2)}\n")
    f.write(f"- 平均LiDAR时间差(ms): {fmt(overall['perf_avg_lidar_delta_ms'], 2)}\n\n")

    f.write("## 三、产物路径\n\n")
    f.write(f"- 三场景目录: {phase_run_dir}\n")
    f.write(f"- 压测目录: {perf_run_dir}\n")

print(md_path)
print(json_path)
PY

if [[ "$PHASE_SOURCE_MODE" != "skipped" && -n "$PHASE_MD" && -f "$PHASE_MD" ]]; then
    cp -f "$PHASE_MD" "$OVERALL_DIR/"
fi
if [[ "$PHASE_SOURCE_MODE" != "skipped" && -n "$PHASE_JSON" && -f "$PHASE_JSON" ]]; then
    cp -f "$PHASE_JSON" "$OVERALL_DIR/"
fi
cp -f "$PERF_REPORT" "$OVERALL_DIR/perf_report.md"

echo "[resume_overall] phase_run_dir=$PHASE_RUN_DIR"
echo "[resume_overall] phase_source_mode=$PHASE_SOURCE_MODE"
echo "[resume_overall] perf_run_dir=$PERF_RUN_DIR"
echo "[resume_overall] summary_md=$OVERALL_MD"
echo "[resume_overall] summary_json=$OVERALL_JSON"
echo "[resume_overall] done"
