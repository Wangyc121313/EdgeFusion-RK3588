#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   scripts/perf_stress_suite.sh [duration_s] [rounds] [output_dir] [-- extra perception_app args...]
# Example:
#   scripts/perf_stress_suite.sh 600 1 reports/perf

DURATION_S="${1:-600}"
ROUNDS="${2:-1}"
OUT_DIR="${3:-reports/perf}"
shift $(( $# >= 3 ? 3 : $# )) || true

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. "$ROOT_DIR/scripts/camera_device_utils.sh"

EXTRA_ARGS=()
if [[ "${1:-}" == "--" ]]; then
    shift
    EXTRA_ARGS=("$@")
fi

APP_BIN="${APP_BIN:-./build/perception_app}"
TARGET_FPS="${TARGET_FPS:-30}"
TELEMETRY_INTERVAL_MS="${TELEMETRY_INTERVAL_MS:-200}"
NET_IFACE="${NET_IFACE:-}"
CPU_COUNT="$(nproc 2>/dev/null || echo 1)"

DEVICE="${DEVICE:-auto}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
MODEL_PATH="${MODEL_PATH:-models/yolov8n.rknn}"
MODEL_W="${MODEL_W:-640}"
MODEL_H="${MODEL_H:-640}"
LABELS_PATH="${LABELS_PATH:-models/coco_80_labels_lists.txt}"
RTSP_URL="${RTSP_URL:-rtsp://0.0.0.0:8554/live/camera}"
FPS="${FPS:-$TARGET_FPS}"
DUMP_H264_PATH="${DUMP_H264_PATH:-}"
INFER_EVERY_N="${INFER_EVERY_N:-5}"
LIDAR_PORT="${LIDAR_PORT:-/dev/ttyUSB0}"
LIDAR_BAUD="${LIDAR_BAUD:-115200}"
export RK3588_CAMERA_FOV_DEG="${RK3588_CAMERA_FOV_DEG:-55}"
LIDAR_OFFSET_DEG="${LIDAR_OFFSET_DEG:-11.7}"
LIDAR_FOV_DEG="${LIDAR_FOV_DEG:-55}"
LIDAR_WINDOW_HALF_DEG="${LIDAR_WINDOW_HALF_DEG:-2.5}"
LIDAR_MIN_DIST_M="${LIDAR_MIN_DIST_M:-0.15}"
LIDAR_MAX_DIST_M="${LIDAR_MAX_DIST_M:-8.0}"
LIDAR_MAX_AGE_MS="${LIDAR_MAX_AGE_MS:-120}"
PUBLISH_MODE="${PUBLISH_MODE:-webrtc}"
WEBRTC_URL="${WEBRTC_URL:-rtc://127.0.0.1:8000/live/camera}"

DEVICE="$(resolve_camera_device "$DEVICE")"
LOCAL_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"

echo "Streaming preview hints:"
if [[ "$PUBLISH_MODE" == "webrtc" || "$PUBLISH_MODE" == "both" ]]; then
    echo "  WebRTC local page: http://127.0.0.1:8080/webrtc/index.html?app=live&stream=camera&type=play"
    if [[ -n "$LOCAL_IP" ]]; then
        echo "  WebRTC LAN page:   http://${LOCAL_IP}:8080/webrtc/index.html?app=live&stream=camera&type=play"
    fi
fi
if [[ "$PUBLISH_MODE" == "rtsp" || "$PUBLISH_MODE" == "both" ]]; then
    RTSP_LOCAL_URL="${RTSP_URL/0.0.0.0/127.0.0.1}"
    echo "  RTSP local URL:    ${RTSP_LOCAL_URL}"
    if [[ -n "$LOCAL_IP" ]]; then
        echo "  RTSP LAN URL:      ${RTSP_URL/0.0.0.0/$LOCAL_IP}"
    fi
fi

DEFAULT_CMD=(
    "$APP_BIN"
    "$DEVICE"
    "$WIDTH"
    "$HEIGHT"
    "$DURATION_S"
    "$MODEL_PATH"
    "$MODEL_W"
    "$MODEL_H"
    "$LABELS_PATH"
    "$RTSP_URL"
    "$FPS"
    "$DUMP_H264_PATH"
    "$INFER_EVERY_N"
    "$LIDAR_PORT"
    "$LIDAR_BAUD"
    "$LIDAR_OFFSET_DEG"
    "$LIDAR_FOV_DEG"
    "$LIDAR_WINDOW_HALF_DEG"
    "$LIDAR_MIN_DIST_M"
    "$LIDAR_MAX_DIST_M"
    "$LIDAR_MAX_AGE_MS"
    "$PUBLISH_MODE"
    "$WEBRTC_URL"
)

if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
    CMD=("$APP_BIN" "${EXTRA_ARGS[@]}")
else
    CMD=("${DEFAULT_CMD[@]}")
fi

if [[ ! -x "$APP_BIN" ]]; then
    echo "perception_app not found, build first:" >&2
    echo "  cmake -S . -B build && cmake --build build -j4 --target perception_app" >&2
    exit 1
fi

mkdir -p "$OUT_DIR"
TS="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$OUT_DIR/run_$TS"
mkdir -p "$RUN_DIR"
ln -sfn "$(basename "$RUN_DIR")" "$OUT_DIR/latest"

detect_net_iface() {
    if [[ -n "$NET_IFACE" && -d "/sys/class/net/$NET_IFACE" ]]; then
        echo "$NET_IFACE"
        return
    fi
    local via
    via="$(ip route 2>/dev/null | awk '/default/ {print $5; exit}' || true)"
    if [[ -n "$via" && -d "/sys/class/net/$via" ]]; then
        echo "$via"
        return
    fi
    echo ""
}

detect_cpu_temp_path() {
    local zone type
    for zone in /sys/class/thermal/thermal_zone*; do
        [[ -f "$zone/type" ]] || continue
        type="$(tr '[:upper:]' '[:lower:]' < "$zone/type" 2>/dev/null || true)"
        if [[ "$type" == *cpu* || "$type" == *soc* || "$type" == *package* ]]; then
            echo "$zone/temp"
            return
        fi
    done
    if [[ -f /sys/class/thermal/thermal_zone0/temp ]]; then
        echo "/sys/class/thermal/thermal_zone0/temp"
        return
    fi
    echo ""
}

detect_npu_load_path() {
    local path
    for path in \
        /sys/kernel/debug/rknpu/load \
        /sys/class/devfreq/*npu*/load \
        /sys/devices/platform/*.npu/devfreq/*/load; do
        if [[ -f "$path" ]]; then
            echo "$path"
            return
        fi
    done
    echo ""
}

detect_npu_freq_path() {
    local path
    for path in \
        /sys/class/devfreq/*npu*/cur_freq \
        /sys/devices/platform/*.npu/devfreq/*/cur_freq; do
        if [[ -f "$path" ]]; then
            echo "$path"
            return
        fi
    done
    echo ""
}

read_proc_io_bytes() {
    local pid="$1"
    awk '/read_bytes/ {r=$2} /write_bytes/ {w=$2} END {print (r+0), (w+0)}' "/proc/$pid/io" 2>/dev/null || echo "0 0"
}

read_proc_ctx_switches() {
    local pid="$1"
    awk '/voluntary_ctxt_switches/ {v=$2} /nonvoluntary_ctxt_switches/ {n=$2} END {print (v+n+0)}' "/proc/$pid/status" 2>/dev/null || echo "0"
}

read_proc_fd_count() {
    local pid="$1"
    ls "/proc/$pid/fd" 2>/dev/null | wc -l | awk '{print $1+0}'
}

read_proc_threads() {
    local pid="$1"
    ls "/proc/$pid/task" 2>/dev/null | wc -l | awk '{print $1+0}'
}

read_net_bytes() {
    local iface="$1"
    if [[ -z "$iface" || ! -d "/sys/class/net/$iface" ]]; then
        echo "0 0"
        return
    fi
    local rx tx
    rx="$(cat "/sys/class/net/$iface/statistics/rx_bytes" 2>/dev/null || echo 0)"
    tx="$(cat "/sys/class/net/$iface/statistics/tx_bytes" 2>/dev/null || echo 0)"
    echo "$rx $tx"
}

read_cpu_freq_khz() {
    local cpu="$1"
    local path="/sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_cur_freq"
    if [[ -f "$path" ]]; then
        cat "$path" 2>/dev/null || echo -1
        return
    fi
    echo -1
}

read_mem_available_mb() {
    awk '/MemAvailable:/ {printf "%.2f", $2/1024.0}' /proc/meminfo 2>/dev/null || echo "-1"
}

read_swap_used_mb() {
    awk '/SwapTotal:/ {t=$2} /SwapFree:/ {f=$2} END {if (t>0) printf "%.2f", (t-f)/1024.0; else print "0.00"}' /proc/meminfo 2>/dev/null
}

read_cpu_temp_c() {
    local path="$1"
    if [[ -n "$path" && -f "$path" ]]; then
        awk '{printf "%.1f", $1/1000.0}' "$path" 2>/dev/null || echo -1
        return
    fi
    echo -1
}

extract_npu_load_pct() {
    local path="$1"
    if [[ -z "$path" || ! -f "$path" ]]; then
        echo -1
        return
    fi
    local raw pct
    raw="$(cat "$path" 2>/dev/null || true)"
    pct="$(echo "$raw" | grep -Eo '[0-9]+' | head -n 1 || true)"
    echo "${pct:--1}"
}

read_npu_freq_khz() {
    local path="$1"
    if [[ -z "$path" || ! -f "$path" ]]; then
        echo -1
        return
    fi
    awk '{printf "%.0f", $1/1000.0}' "$path" 2>/dev/null || echo -1
}

sample_runtime_stats() {
    local pid="$1"
    local csv="$2"
    local iface="$3"
    local cpu_temp_path="$4"
    local npu_load_path="$5"
    local npu_freq_path="$6"

    echo "sec,cpu_total_pct,cpu_proc_pct,rss_mb,vsz_mb,threads,fd_count,mem_available_mb,swap_used_mb,temp_cpu_c,cpu0_freq_khz,cpu4_freq_khz,cpu7_freq_khz,io_read_kbps,io_write_kbps,ctx_switches_ps,net_rx_kbps,net_tx_kbps,rknpu_pct,rknpu_freq_khz" > "$csv"

    local pagesize prev_proc=0 prev_total=0 prev_idle=0 prev_read=0 prev_write=0 prev_ctx=0 prev_rx=0 prev_tx=0 sec=0
    pagesize="$(getconf PAGESIZE)"

    read -r prev_read prev_write <<< "$(read_proc_io_bytes "$pid")"
    prev_ctx="$(read_proc_ctx_switches "$pid")"
    read -r prev_rx prev_tx <<< "$(read_net_bytes "$iface")"
    read -r prev_total prev_idle <<< "$(awk '/^cpu /{idle=$5+$6; total=0; for(i=2;i<=NF;++i) total+=$i; print total, idle}' /proc/stat 2>/dev/null || echo '0 0')"

    while kill -0 "$pid" 2>/dev/null; do
        local proc_stat cpu_stat utime stime vsz_bytes rss_pages threads_stat
        proc_stat="$(awk '{print $14, $15, $23, $24, $20}' "/proc/$pid/stat" 2>/dev/null || true)"
        cpu_stat="$(awk '/^cpu /{idle=$5+$6; total=0; for(i=2;i<=NF;++i) total+=$i; print total, idle}' /proc/stat 2>/dev/null || true)"
        if [[ -z "$proc_stat" || -z "$cpu_stat" ]]; then
            sleep 1
            ((sec++)) || true
            continue
        fi

        read -r utime stime vsz_bytes rss_pages threads_stat <<< "$proc_stat"
        local proc_total=$((utime + stime))
        local cpu_total cpu_idle
        read -r cpu_total cpu_idle <<< "$cpu_stat"

        local cpu_total_pct="0.00"
        local cpu_proc_pct="0.00"
        if [[ "$prev_total" -gt 0 ]]; then
            local d_total=$((cpu_total - prev_total))
            local d_idle=$((cpu_idle - prev_idle))
            local d_proc=$((proc_total - prev_proc))
            if [[ "$d_total" -gt 0 ]]; then
                cpu_total_pct="$(awk -v idle="$d_idle" -v total="$d_total" 'BEGIN{printf "%.2f", (1.0 - idle/total) * 100.0}')"
                cpu_proc_pct="$(awk -v proc="$d_proc" -v total="$d_total" -v ncpu="$CPU_COUNT" 'BEGIN{printf "%.2f", (proc * 100.0 * ncpu) / total}')"
            fi
        fi
        prev_total="$cpu_total"
        prev_idle="$cpu_idle"
        prev_proc="$proc_total"

        local rss_mb vsz_mb
        rss_mb="$(awk -v p="$rss_pages" -v s="$pagesize" 'BEGIN{printf "%.2f", (p*s)/(1024*1024)}')"
        vsz_mb="$(awk -v b="$vsz_bytes" 'BEGIN{printf "%.2f", b/(1024*1024)}')"

        local threads fd_count mem_available_mb swap_used_mb temp_cpu_c
        threads="$(read_proc_threads "$pid")"
        if [[ "$threads" -le 0 ]]; then
            threads="$threads_stat"
        fi
        fd_count="$(read_proc_fd_count "$pid")"
        mem_available_mb="$(read_mem_available_mb)"
        swap_used_mb="$(read_swap_used_mb)"
        temp_cpu_c="$(read_cpu_temp_c "$cpu_temp_path")"

        local io_read io_write d_read d_write io_read_kbps io_write_kbps
        read -r io_read io_write <<< "$(read_proc_io_bytes "$pid")"
        d_read=$((io_read - prev_read))
        d_write=$((io_write - prev_write))
        prev_read="$io_read"
        prev_write="$io_write"
        if [[ "$d_read" -lt 0 ]]; then d_read=0; fi
        if [[ "$d_write" -lt 0 ]]; then d_write=0; fi
        io_read_kbps="$(awk -v b="$d_read" 'BEGIN{printf "%.2f", b/1024.0}')"
        io_write_kbps="$(awk -v b="$d_write" 'BEGIN{printf "%.2f", b/1024.0}')"

        local ctx_now d_ctx rx_now tx_now d_rx d_tx net_rx_kbps net_tx_kbps
        ctx_now="$(read_proc_ctx_switches "$pid")"
        d_ctx=$((ctx_now - prev_ctx))
        prev_ctx="$ctx_now"
        if [[ "$d_ctx" -lt 0 ]]; then d_ctx=0; fi

        read -r rx_now tx_now <<< "$(read_net_bytes "$iface")"
        d_rx=$((rx_now - prev_rx))
        d_tx=$((tx_now - prev_tx))
        prev_rx="$rx_now"
        prev_tx="$tx_now"
        if [[ "$d_rx" -lt 0 ]]; then d_rx=0; fi
        if [[ "$d_tx" -lt 0 ]]; then d_tx=0; fi
        net_rx_kbps="$(awk -v b="$d_rx" 'BEGIN{printf "%.2f", b/1024.0}')"
        net_tx_kbps="$(awk -v b="$d_tx" 'BEGIN{printf "%.2f", b/1024.0}')"

        local cpu0_freq_khz cpu4_freq_khz cpu7_freq_khz rknpu_pct rknpu_freq_khz
        cpu0_freq_khz="$(read_cpu_freq_khz 0)"
        cpu4_freq_khz="$(read_cpu_freq_khz 4)"
        cpu7_freq_khz="$(read_cpu_freq_khz 7)"
        rknpu_pct="$(extract_npu_load_pct "$npu_load_path")"
        rknpu_freq_khz="$(read_npu_freq_khz "$npu_freq_path")"

        echo "$sec,$cpu_total_pct,$cpu_proc_pct,$rss_mb,$vsz_mb,$threads,$fd_count,$mem_available_mb,$swap_used_mb,$temp_cpu_c,$cpu0_freq_khz,$cpu4_freq_khz,$cpu7_freq_khz,$io_read_kbps,$io_write_kbps,$d_ctx,$net_rx_kbps,$net_tx_kbps,$rknpu_pct,$rknpu_freq_khz" >> "$csv"

        sleep 1
        ((sec++)) || true
    done
}

summarize_round() {
    local log_file="$1"
    local summary_file="$2"
    local exit_code="$3"
    local final_line input_frames output_packets encode_frames run_seconds avg_fps fps_util_pct packet_per_frame frame_delivery_ratio

    final_line="$(grep -E 'done: input_frames=' "$log_file" | tail -n 1 || true)"
    input_frames="$(echo "$final_line" | sed -n 's/.*input_frames=\([0-9]\+\).*/\1/p')"
    output_packets="$(echo "$final_line" | sed -n 's/.*output_packets=\([0-9]\+\).*/\1/p')"
    encode_frames="$(echo "$final_line" | sed -n 's/.*encode_frames=\([0-9]\+\).*/\1/p')"
    run_seconds="$(echo "$final_line" | sed -n 's/.*run_seconds=\([-0-9]\+\).*/\1/p')"

    input_frames="${input_frames:-0}"
    output_packets="${output_packets:-0}"
    encode_frames="${encode_frames:-0}"
    run_seconds="${run_seconds:-0}"

    avg_fps="0.00"
    if [[ "$run_seconds" -gt 0 ]]; then
        avg_fps="$(awk -v f="$encode_frames" -v t="$run_seconds" 'BEGIN{printf "%.2f", f/t}')"
    fi

    packet_per_frame="0.000"
    frame_delivery_ratio="0.000"
    if [[ "$input_frames" -gt 0 ]]; then
        packet_per_frame="$(awk -v o="$output_packets" -v i="$input_frames" 'BEGIN{printf "%.3f", o/i}')"
        frame_delivery_ratio="$(awk -v o="$output_packets" -v i="$input_frames" 'BEGIN{r=o/i; if(r>1) r=1; if(r<0) r=0; printf "%.3f", r}')"
    fi

    fps_util_pct="0.00"
    if [[ "$TARGET_FPS" -gt 0 ]]; then
        fps_util_pct="$(awk -v a="$avg_fps" -v t="$TARGET_FPS" 'BEGIN{printf "%.2f", (a*100.0)/t}')"
    fi

    {
        echo "exit_code=$exit_code"
        echo "final_line=$final_line"
        echo "input_frames=$input_frames"
        echo "output_packets=$output_packets"
        echo "encode_frames=$encode_frames"
        echo "run_seconds=$run_seconds"
        echo "avg_fps=$avg_fps"
        echo "target_fps=$TARGET_FPS"
        echo "fps_util_pct=$fps_util_pct"
        echo "packet_per_frame=$packet_per_frame"
        echo "frame_delivery_ratio=$frame_delivery_ratio"
    } > "$summary_file"
}

generate_round_kpis() {
    local runtime_csv="$1"
    local telemetry_jsonl="$2"
    local summary_file="$3"
    local kpi_csv="$4"
    local report_md="$5"

    python3 - "$runtime_csv" "$telemetry_jsonl" "$summary_file" "$kpi_csv" "$report_md" <<'PY'
import csv
import json
import math
import os
import statistics
import sys

runtime_csv, telemetry_jsonl, summary_file, kpi_csv, report_md = sys.argv[1:6]


def load_summary(path):
    data = {}
    if not os.path.exists(path):
        return data
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.rstrip("\n")
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            data[key] = value
    return data


def to_float(value):
    try:
        if value in (None, "", "n/a"):
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def percentile(values, ratio):
    if not values:
        return None
    ordered = sorted(values)
    index = int((len(ordered) - 1) * ratio)
    return ordered[index]


def stats(values):
    cleaned = [float(v) for v in values if v is not None and not math.isnan(float(v))]
    if not cleaned:
        return None
    mean = statistics.fmean(cleaned)
    std = statistics.pstdev(cleaned) if len(cleaned) > 1 else 0.0
    p90 = percentile(cleaned, 0.90)
    return mean, std, p90


def fmt(value, digits=3):
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}"


runtime_rows = []
if os.path.exists(runtime_csv):
    with open(runtime_csv, "r", encoding="utf-8") as handle:
        runtime_rows = list(csv.DictReader(handle))

telemetry_rows = []
if os.path.exists(telemetry_jsonl):
    with open(telemetry_jsonl, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                telemetry_rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue

summary = load_summary(summary_file)


def floats_from_runtime(key, minimum=None):
    values = []
    for row in runtime_rows:
        value = to_float(row.get(key))
        if value is None:
            continue
        if minimum is not None and value < minimum:
            continue
        values.append(value)
    return values


def floats_from_telemetry(key, minimum=None):
    values = []
    for row in telemetry_rows:
        value = to_float(row.get(key))
        if value is None:
            continue
        if minimum is not None and value < minimum:
            continue
        values.append(value)
    return values


runtime_metrics = [
    ("cpu_total_pct", "系统总 CPU 占用率", floats_from_runtime("cpu_total_pct")),
    ("cpu_proc_pct", "感知进程 CPU 占用率", floats_from_runtime("cpu_proc_pct")),
    ("rss_mb", "进程常驻内存 RSS（MB）", floats_from_runtime("rss_mb")),
    ("vsz_mb", "进程虚拟内存 VSZ（MB）", floats_from_runtime("vsz_mb")),
    ("mem_available_mb", "系统可用内存（MB）", floats_from_runtime("mem_available_mb")),
    ("swap_used_mb", "系统已用 Swap（MB）", floats_from_runtime("swap_used_mb")),
    ("temp_cpu_c", "CPU 温度（摄氏度）", floats_from_runtime("temp_cpu_c", 0.0)),
    ("cpu0_freq_khz", "CPU0 频率（kHz）", floats_from_runtime("cpu0_freq_khz", 0.0)),
    ("cpu4_freq_khz", "CPU4 频率（kHz）", floats_from_runtime("cpu4_freq_khz", 0.0)),
    ("cpu7_freq_khz", "CPU7 频率（kHz）", floats_from_runtime("cpu7_freq_khz", 0.0)),
    ("net_tx_kbps", "网络发送带宽（KB/s）", floats_from_runtime("net_tx_kbps")),
    ("ctx_switches_ps", "进程每秒上下文切换次数", floats_from_runtime("ctx_switches_ps")),
    ("rknpu_pct", "RKNPU/NPU 负载（%）", floats_from_runtime("rknpu_pct", 0.0)),
    ("rknpu_freq_khz", "RKNPU/NPU 频率（kHz）", floats_from_runtime("rknpu_freq_khz", 0.0)),
]

fps_out_inst = []
closest_distance = []
closest_ttc = []
distance_ready_ratio = []
confirmed_ratio = []
lidar_match_pct = []
infer_hit_ratio = []
lidar_delta = []
for prev, cur in zip(telemetry_rows, telemetry_rows[1:]):
    prev_ts = to_float(prev.get("ts_ms"))
    cur_ts = to_float(cur.get("ts_ms"))
    prev_frames = to_float(prev.get("encode_frames"))
    cur_frames = to_float(cur.get("encode_frames"))
    if prev_ts is not None and cur_ts is not None and cur_ts > prev_ts and prev_frames is not None and cur_frames is not None:
        fps_out_inst.append((cur_frames - prev_frames) * 1000.0 / (cur_ts - prev_ts))

for row in telemetry_rows:
    target_count = to_float(row.get("target_count")) or 0.0
    confirmed_count = to_float(row.get("confirmed_target_count")) or 0.0
    valid_distance_count = to_float(row.get("valid_distance_target_count")) or 0.0
    if target_count > 0.0:
        distance_ready_ratio.append(valid_distance_count * 100.0 / target_count)
        confirmed_ratio.append(confirmed_count * 100.0 / target_count)
    lidar_match_pct.append(100.0 if row.get("lidar_matched") else 0.0)
    infer_hit_ratio.append(100.0 if row.get("did_infer") else 0.0)
    if row.get("lidar_matched"):
        value = to_float(row.get("lidar_delta_ms"))
        if value is not None:
            lidar_delta.append(value)

    distances = [to_float(t.get("distance_m")) for t in row.get("targets", [])]
    distances = [d for d in distances if d is not None and d >= 0.0]
    if distances:
        closest_distance.append(min(distances))

    ttcs = [to_float(t.get("ttc_s")) for t in row.get("targets", [])]
    ttcs = [t for t in ttcs if t is not None and t > 0.0]
    if ttcs:
        closest_ttc.append(min(ttcs))

telemetry_metrics = [
    ("capture_to_encode_ms", "应用内端到端时延（采集时间戳到编码提交）", floats_from_telemetry("capture_to_encode_ms")),
    ("preprocess_ms", "单次前处理耗时（RGA resize / CSC）", [v for v in floats_from_telemetry("preprocess_ms") if v > 0.0]),
    ("infer_ms", "单次 RKNN 推理耗时（仅统计实际推理帧）", [v for v in floats_from_telemetry("infer_ms") if v > 0.0]),
    ("fusion_ms", "单次视觉-LiDAR 距离融合耗时", floats_from_telemetry("fusion_ms")),
    ("track_ms", "单次多目标跟踪耗时", floats_from_telemetry("track_ms")),
    ("overlay_ms", "单次叠框/HUD 耗时", floats_from_telemetry("overlay_ms")),
    ("encode_submit_ms", "单次编码提交耗时", floats_from_telemetry("encode_submit_ms")),
    ("fps_out_inst", "基于 telemetry 相邻样本估算的输出瞬时帧率", fps_out_inst),
    ("encode_fps", "累计编码输出帧率", floats_from_telemetry("encode_fps")),
    ("target_count", "单个 telemetry 样本中的目标数量", floats_from_telemetry("target_count")),
    ("confirmed_target_count", "已确认跟踪目标数量", floats_from_telemetry("confirmed_target_count")),
    ("valid_distance_target_count", "已得到有效距离的目标数量", floats_from_telemetry("valid_distance_target_count")),
    ("distance_ready_ratio_pct", "目标中成功给出距离的比例（%）", distance_ready_ratio),
    ("confirmed_target_ratio_pct", "目标中已确认跟踪的比例（%）", confirmed_ratio),
    ("closest_target_distance_m", "最近目标距离（米）", closest_distance),
    ("closest_ttc_s", "最近 TTC（秒）", closest_ttc),
    ("ttc_alert_count", "TTC<=3s 的告警目标数量", floats_from_telemetry("ttc_alert_count")),
    ("lidar_match_pct", "样本中匹配到 LiDAR 点云的比例（%）", lidar_match_pct),
    ("lidar_delta_ms", "视频帧与匹配 LiDAR 扫描的时间差", lidar_delta),
    ("lidar_scan_period_ms", "LiDAR 估计扫描周期（ms）", [v for v in floats_from_telemetry("lidar_scan_period_ms") if v > 0.0]),
    ("lidar_points_total", "单个匹配点云中的点数", [v for v in floats_from_telemetry("lidar_points_total") if v > 0.0]),
    ("infer_hit_ratio_pct", "Telemetry 采样点中执行 RKNN 推理的比例（%）", infer_hit_ratio),
]

rows = []
for name, meaning, values in runtime_metrics + telemetry_metrics:
    stat = stats(values)
    if stat is None:
        rows.append((name, meaning, "n/a", "n/a", "n/a"))
    else:
        mean, std, p90 = stat
        rows.append((name, meaning, fmt(mean), fmt(std), fmt(p90)))

with open(kpi_csv, "w", encoding="utf-8", newline="") as handle:
    writer = csv.writer(handle)
    writer.writerow(["metric", "meaning", "mean", "std", "p90"])
    writer.writerows(rows)

with open(report_md, "w", encoding="utf-8") as handle:
    handle.write("| 指标 | 含义 | mean / std / p90 |\n")
    handle.write("|---|---|---|\n")
    for metric, meaning, mean, std, p90 in rows:
        handle.write(f"| `{metric}` | {meaning} | {mean} / {std} / {p90} |\n")
    handle.write("\n")
    handle.write("稳定性补充：")
    handle.write(
        f"本轮 status={'ok' if summary.get('exit_code') == '0' else 'failed'}，"
        f"run_seconds={summary.get('run_seconds', '0')}，"
        f"input_frames={summary.get('input_frames', '0')}，"
        f"encode_frames={summary.get('encode_frames', '0')}，"
        f"fps_util_pct={summary.get('fps_util_pct', '0')}。\n"
    )
PY
}

read_metric_mean() {
    local csv_file="$1"
    local metric="$2"
    awk -F, -v key="$metric" 'NR>1 && $1==key {print $3; exit}' "$csv_file"
}

echo "[suite] output dir: $RUN_DIR"
echo "[suite] rounds=$ROUNDS duration_s=$DURATION_S telemetry_interval_ms=$TELEMETRY_INTERVAL_MS"
IFACE="$(detect_net_iface)"
CPU_TEMP_PATH="$(detect_cpu_temp_path)"
NPU_LOAD_PATH="$(detect_npu_load_path)"
NPU_FREQ_PATH="$(detect_npu_freq_path)"
echo "[suite] net_if=${IFACE:-N/A} target_fps=$TARGET_FPS"
echo "[suite] cpu_temp=${CPU_TEMP_PATH:-N/A} npu_load=${NPU_LOAD_PATH:-N/A} npu_freq=${NPU_FREQ_PATH:-N/A}"
echo "[suite] cmd: ${CMD[*]}"

for ((i=1; i<=ROUNDS; ++i)); do
    ROUND_DIR="$RUN_DIR/round_$i"
    mkdir -p "$ROUND_DIR"

    LOG_FILE="$ROUND_DIR/perception.log"
    CSV_FILE="$ROUND_DIR/runtime.csv"
    TEL_FILE="$ROUND_DIR/telemetry.jsonl"
    SUM_FILE="$ROUND_DIR/summary.txt"
    KPI_CSV="$ROUND_DIR/kpi_summary.csv"
    KPI_MD="$ROUND_DIR/kpi_report.md"

    echo "[suite] round $i/$ROUNDS start"

    set +e
    RK3588_TELEMETRY_PATH="$TEL_FILE" \
    RK3588_TELEMETRY_INTERVAL_MS="$TELEMETRY_INTERVAL_MS" \
    RK3588_DEBUG_VIDEO_HUD=0 \
    "${CMD[@]}" > "$LOG_FILE" 2>&1 &
    PID=$!
    set -e

    sample_runtime_stats "$PID" "$CSV_FILE" "$IFACE" "$CPU_TEMP_PATH" "$NPU_LOAD_PATH" "$NPU_FREQ_PATH"

    set +e
    wait "$PID"
    EXIT_CODE=$?
    set -e

    summarize_round "$LOG_FILE" "$SUM_FILE" "$EXIT_CODE"
    generate_round_kpis "$CSV_FILE" "$TEL_FILE" "$SUM_FILE" "$KPI_CSV" "$KPI_MD"

    echo "[suite] round $i summary:"
    cat "$SUM_FILE"
    echo

done

GLOBAL_SUM="$RUN_DIR/overall_summary.csv"
echo "round,exit_code,avg_fps,fps_util_pct,cpu_total_pct_mean,cpu_proc_pct_mean,rss_mb_mean,temp_cpu_c_mean,rknpu_pct_mean,encode_fps_mean,capture_to_encode_ms_mean,infer_ms_mean,lidar_match_pct_mean,lidar_delta_ms_mean,lidar_points_total_mean" > "$GLOBAL_SUM"
for ((i=1; i<=ROUNDS; ++i)); do
    SUM_FILE="$RUN_DIR/round_$i/summary.txt"
    KPI_CSV="$RUN_DIR/round_$i/kpi_summary.csv"
    exit_code="$(grep '^exit_code=' "$SUM_FILE" | cut -d= -f2)"
    avg_fps="$(grep '^avg_fps=' "$SUM_FILE" | cut -d= -f2)"
    fps_util_pct="$(grep '^fps_util_pct=' "$SUM_FILE" | cut -d= -f2)"
    cpu_total_mean="$(read_metric_mean "$KPI_CSV" cpu_total_pct)"
    cpu_proc_mean="$(read_metric_mean "$KPI_CSV" cpu_proc_pct)"
    rss_mean="$(read_metric_mean "$KPI_CSV" rss_mb)"
    temp_mean="$(read_metric_mean "$KPI_CSV" temp_cpu_c)"
    npu_mean="$(read_metric_mean "$KPI_CSV" rknpu_pct)"
    encode_fps_mean="$(read_metric_mean "$KPI_CSV" encode_fps)"
    e2e_mean="$(read_metric_mean "$KPI_CSV" capture_to_encode_ms)"
    infer_mean="$(read_metric_mean "$KPI_CSV" infer_ms)"
    lidar_match_mean="$(read_metric_mean "$KPI_CSV" lidar_match_pct)"
    lidar_delta_mean="$(read_metric_mean "$KPI_CSV" lidar_delta_ms)"
    lidar_points_mean="$(read_metric_mean "$KPI_CSV" lidar_points_total)"
    echo "$i,$exit_code,$avg_fps,$fps_util_pct,$cpu_total_mean,$cpu_proc_mean,$rss_mean,$temp_mean,$npu_mean,$encode_fps_mean,$e2e_mean,$infer_mean,$lidar_match_mean,$lidar_delta_mean,$lidar_points_mean" >> "$GLOBAL_SUM"
done

REPORT_MD="$RUN_DIR/report.md"
{
    echo "# Performance Report"
    echo
    echo "> 参考 RK-MediaProject 的 KPI 展示方式，按轮次输出系统指标、视觉/RKNN 链路指标、LiDAR 匹配质量指标和稳定性补充。"
    echo
    echo "- app: $APP_BIN"
    echo "- rounds: $ROUNDS"
    echo "- duration_s_each: $DURATION_S"
    echo "- telemetry_interval_ms: $TELEMETRY_INTERVAL_MS"
    echo "- target_fps: $TARGET_FPS"
    echo "- net_iface: ${IFACE:-N/A}"
    echo "- cpu_temp_path: ${CPU_TEMP_PATH:-N/A}"
    echo "- npu_load_path: ${NPU_LOAD_PATH:-N/A}"
    echo "- npu_freq_path: ${NPU_FREQ_PATH:-N/A}"
    echo
    echo "## Overall Summary"
    echo
    echo '```csv'
    cat "$GLOBAL_SUM"
    echo '```'
    echo
    for ((i=1; i<=ROUNDS; ++i)); do
        echo "## Round $i"
        echo
        cat "$RUN_DIR/round_$i/kpi_report.md"
        echo
        echo "Artifacts:"
        echo "- log: round_$i/perception.log"
        echo "- runtime samples: round_$i/runtime.csv"
        echo "- telemetry samples: round_$i/telemetry.jsonl"
        echo "- key-value summary: round_$i/summary.txt"
        echo "- KPI csv: round_$i/kpi_summary.csv"
        echo
    done
} > "$REPORT_MD"

echo "[suite] done. overall summary: $GLOBAL_SUM"
echo "[suite] markdown report: $REPORT_MD"
