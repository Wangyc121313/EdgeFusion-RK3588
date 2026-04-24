#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

CAMERA_DEVICE="${1:-/dev/video0}"
CAMERA_WIDTH="${2:-640}"
CAMERA_HEIGHT="${3:-480}"
RUN_SECONDS="${4:-75}"
OUT_ROOT="${5:-reports/phasec}"

APP_BIN="${APP_BIN:-./build/perception_app}"
MODEL_PATH="${MODEL_PATH:-models/yolov8n.rknn}"
MODEL_W="${MODEL_W:-640}"
MODEL_H="${MODEL_H:-640}"
LABELS_PATH="${LABELS_PATH:-third_party/rknn_model_zoo/examples/yolov8/model/coco_80_labels_list.txt}"
RTSP_URL="${RTSP_URL:-rtsp://127.0.0.1:8554/live/camera}"
FPS="${FPS:-30}"
INFER_EVERY_N="${INFER_EVERY_N:-5}"
LIDAR_PORT="${LIDAR_PORT:-/dev/ttyUSB0}"
LIDAR_BAUD="${LIDAR_BAUD:-115200}"
LIDAR_OFFSET_DEG="${LIDAR_OFFSET_DEG:-11.7}"
LIDAR_FOV_DEG="${LIDAR_FOV_DEG:-55}"
LIDAR_WINDOW_HALF_DEG="${LIDAR_WINDOW_HALF_DEG:-2.5}"
LIDAR_MIN_DIST_M="${LIDAR_MIN_DIST_M:-0.15}"
LIDAR_MAX_DIST_M="${LIDAR_MAX_DIST_M:-20.0}"
LIDAR_MAX_AGE_MS="${LIDAR_MAX_AGE_MS:-120}"
PUBLISH_MODE="${PUBLISH_MODE:-webrtc}"
WEBRTC_URL="${WEBRTC_URL:-rtc://127.0.0.1:8000/live/camera}"
START_DEBUG_UI="${START_DEBUG_UI:-1}"
DEBUG_UI_HOST="${DEBUG_UI_HOST:-0.0.0.0}"
DEBUG_UI_PORT="${DEBUG_UI_PORT:-8090}"
DEBUG_UI_ZLM_HOST="${DEBUG_UI_ZLM_HOST:-127.0.0.1}"
DEBUG_UI_LOG_PATH="${DEBUG_UI_LOG_PATH:-/tmp/rk3588_phasec_debug_ui.log}"
DEBUG_UI_BIN="${DEBUG_UI_BIN:-$ROOT_DIR/build/webrtc_debug_ui_server}"

export RK3588_WEBRTC_RTC_PORT="${RK3588_WEBRTC_RTC_PORT:-8000}"
export RK3588_WEBRTC_HTTP_PORT="${RK3588_WEBRTC_HTTP_PORT:-8080}"

export RK3588_DISTANCE_FUSION_MODE="${RK3588_DISTANCE_FUSION_MODE:-robust}"
export RK3588_TRACKER_MIN_IOU="${RK3588_TRACKER_MIN_IOU:-0.08}"
export RK3588_TRACKER_IOU_WEIGHT="${RK3588_TRACKER_IOU_WEIGHT:-0.40}"
export RK3588_TRACKER_GHOST_KEEP_FRAMES="${RK3588_TRACKER_GHOST_KEEP_FRAMES:-4}"
export RK3588_TRACKER_MAX_IDLE_FRAMES="${RK3588_TRACKER_MAX_IDLE_FRAMES:-12}"
export RK3588_TRACKER_CENTER_VEL_ALPHA="${RK3588_TRACKER_CENTER_VEL_ALPHA:-0.28}"
export RK3588_TRACKER_GHOST_DECAY="${RK3588_TRACKER_GHOST_DECAY:-0.78}"
export RK3588_PSEUDO_LABEL_MAX_LINES="${RK3588_PSEUDO_LABEL_MAX_LINES:-5000}"

if [[ ! -x "$APP_BIN" ]]; then
  echo "error: $APP_BIN not found, please build first"
  exit 2
fi

RUN_TS="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$OUT_ROOT/run_$RUN_TS"
mkdir -p "$RUN_DIR"
ln -sfn "run_$RUN_TS" "$OUT_ROOT/latest"
TELEMETRY_LINK="$RUN_DIR/current_telemetry.jsonl"

echo "[phasec] run_dir=$RUN_DIR"
echo "[phasec] publish_mode=$PUBLISH_MODE"

DEBUG_UI_PID=""
cleanup() {
  if [[ -n "$DEBUG_UI_PID" ]]; then
    kill "$DEBUG_UI_PID" 2>/dev/null || true
    wait "$DEBUG_UI_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$START_DEBUG_UI" == "1" ]]; then
  if command -v ss >/dev/null 2>&1 && ss -ltnu | awk '{print $5}' | grep -Eq "[:.]${DEBUG_UI_PORT}$"; then
    echo "[phasec] debug ui port ${DEBUG_UI_PORT} already in use, reuse existing service"
  else
    if [[ -x "$DEBUG_UI_BIN" ]]; then
      nohup "$DEBUG_UI_BIN" \
        --host "$DEBUG_UI_HOST" \
        --port "$DEBUG_UI_PORT" \
        --zlm-host "$DEBUG_UI_ZLM_HOST" \
        --zlm-http-port "$RK3588_WEBRTC_HTTP_PORT" \
        --telemetry-path "$TELEMETRY_LINK" \
        --default-app live \
        --default-stream camera \
        >"$DEBUG_UI_LOG_PATH" 2>&1 &
    else
      nohup python3 "$ROOT_DIR/tools/webrtc_debug_ui/server.py" \
        --host "$DEBUG_UI_HOST" \
        --port "$DEBUG_UI_PORT" \
        --zlm-host "$DEBUG_UI_ZLM_HOST" \
        --zlm-http-port "$RK3588_WEBRTC_HTTP_PORT" \
        --telemetry-path "$TELEMETRY_LINK" \
        --default-app live \
        --default-stream camera \
        >"$DEBUG_UI_LOG_PATH" 2>&1 &
    fi
    DEBUG_UI_PID=$!
    echo "[phasec] debug_ui=http://127.0.0.1:${DEBUG_UI_PORT}/?app=live&stream=camera"
    echo "[phasec] debug_ui_log=$DEBUG_UI_LOG_PATH"
  fi
fi

declare -A SCENE_DESC
SCENE_DESC[static_target]="静止目标：目标固定不动，设备尽量保持静止"
SCENE_DESC[approaching_target]="接近目标：目标缓慢接近设备并保持在视场内"
SCENE_DESC[crossing_occlusion]="交叉遮挡：两个目标交叉通过，制造短时遮挡"

SCENES=(static_target approaching_target crossing_occlusion)

for SCENE in "${SCENES[@]}"; do
  SCENE_DIR="$RUN_DIR/$SCENE"
  mkdir -p "$SCENE_DIR"

  export RK3588_PSEUDO_LABEL_SEQUENCE_ID="${RUN_TS}_${SCENE}"
  export RK3588_PSEUDO_LABEL_PATH="$SCENE_DIR/pseudo_labels.jsonl"
  export RK3588_TELEMETRY_PATH="$SCENE_DIR/telemetry.jsonl"
  ln -sfn "$SCENE_DIR/telemetry.jsonl" "$TELEMETRY_LINK"
  VIDEO_PATH="$SCENE_DIR/video.h264"

  {
    echo "RK3588_PSEUDO_LABEL_SEQUENCE_ID=$RK3588_PSEUDO_LABEL_SEQUENCE_ID"
    echo "RK3588_PSEUDO_LABEL_PATH=$RK3588_PSEUDO_LABEL_PATH"
    echo "RK3588_TELEMETRY_PATH=$RK3588_TELEMETRY_PATH"
    echo "RK3588_DISTANCE_FUSION_MODE=$RK3588_DISTANCE_FUSION_MODE"
    echo "RK3588_TRACKER_MIN_IOU=$RK3588_TRACKER_MIN_IOU"
    echo "RK3588_TRACKER_IOU_WEIGHT=$RK3588_TRACKER_IOU_WEIGHT"
    echo "RK3588_TRACKER_GHOST_KEEP_FRAMES=$RK3588_TRACKER_GHOST_KEEP_FRAMES"
    echo "RK3588_TRACKER_MAX_IDLE_FRAMES=$RK3588_TRACKER_MAX_IDLE_FRAMES"
    echo "RK3588_TRACKER_CENTER_VEL_ALPHA=$RK3588_TRACKER_CENTER_VEL_ALPHA"
    echo "RK3588_TRACKER_GHOST_DECAY=$RK3588_TRACKER_GHOST_DECAY"
  } > "$SCENE_DIR/config.env"

  echo
  echo "[scene] $SCENE"
  echo "[desc ] ${SCENE_DESC[$SCENE]}"
  if [[ "${PHASEC_NO_PAUSE:-0}" != "1" ]]; then
    read -r -p "准备好后按 Enter 开始采集..."
  fi

  "$APP_BIN" \
    "$CAMERA_DEVICE" \
    "$CAMERA_WIDTH" \
    "$CAMERA_HEIGHT" \
    "$RUN_SECONDS" \
    "$MODEL_PATH" \
    "$MODEL_W" \
    "$MODEL_H" \
    "$LABELS_PATH" \
    "$RTSP_URL" \
    "$FPS" \
    "$VIDEO_PATH" \
    "$INFER_EVERY_N" \
    "$LIDAR_PORT" \
    "$LIDAR_BAUD" \
    "$LIDAR_OFFSET_DEG" \
    "$LIDAR_FOV_DEG" \
    "$LIDAR_WINDOW_HALF_DEG" \
    "$LIDAR_MIN_DIST_M" \
    "$LIDAR_MAX_DIST_M" \
    "$LIDAR_MAX_AGE_MS" \
    "$PUBLISH_MODE" \
    "$WEBRTC_URL"

  python3 tools/diagnostics/validate_pseudo_labels.py --glob "$SCENE_DIR/pseudo_labels.jsonl*" > "$SCENE_DIR/validate_summary.txt"
  python3 tools/diagnostics/analyze_tracking_metrics.py --glob "$SCENE_DIR/pseudo_labels.jsonl*" > "$SCENE_DIR/tracking_metrics.txt"
done

python3 tools/diagnostics/analyze_phasec_metrics.py --run-dir "$RUN_DIR" --markdown-out "$RUN_DIR/metrics_table.md" --csv-out "$RUN_DIR/metrics_table.csv" > "$RUN_DIR/phasec_metrics.txt"

scripts/build_phasec_showcase.sh "$RUN_DIR" > "$RUN_DIR/showcase_build.txt"

echo "[phasec] done"
echo "[phasec] metrics: $RUN_DIR/metrics_table.md"
echo "[phasec] showcase: $RUN_DIR/showcase"
