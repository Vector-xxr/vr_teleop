#!/usr/bin/env bash
# Restart Quest bridge + isolated camera MJPEG process (/dev/video16 by default).
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_NAME="${OPENARM_QUEST_CONDA_ENV:-openarm_quest}"

if command -v conda >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source "$(conda info --base)/etc/profile.d/conda.sh"
elif [[ -x "${HOME}/miniconda3/bin/conda" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/miniconda3/etc/profile.d/conda.sh"
fi
conda activate "${ENV_NAME}"

CAM="${OPENARM_QUEST_CAMERA:-16}"
CAM_PORT="${OPENARM_QUEST_CAMERA_PORT:-8444}"
HTTP_PORT="${OPENARM_QUEST_HTTP_PORT:-8443}"
# Local OpenCV window while teleop (same V4L2 capture as Quest MJPEG).
export OPENARM_QUEST_CAMERA_PREVIEW="${OPENARM_QUEST_CAMERA_PREVIEW:-1}"
export OPENARM_QUEST_CAMERA_PREVIEW_SCALE="${OPENARM_QUEST_CAMERA_PREVIEW_SCALE:-2.0}"

if command -v fuser >/dev/null 2>&1 && [[ "${CAM}" =~ ^[0-9]+$ ]]; then
  if fuser "/dev/video${CAM}" >/dev/null 2>&1; then
    echo "[quest-cam] /dev/video${CAM} is busy:"
    fuser -v "/dev/video${CAM}" 2>&1 || true
    echo "[quest-cam][HINT] close ffplay/Cheese, or: fuser -k /dev/video${CAM}"
  fi
fi

pkill -f '[q]uest_bridge.py' 2>/dev/null || true
pkill -f '[c]amera_mjpeg_https.py' 2>/dev/null || true
sleep 0.5

LAN_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "[quest-cam] camera=${CAM}  cam_port=${CAM_PORT}"
echo "[quest-cam] Quest UI : https://${LAN_IP:-<LAN-IP>}:${HTTP_PORT}/client.html"
echo "[quest-cam] FPV URL  : https://${LAN_IP:-<LAN-IP>}:${CAM_PORT}/cam.mjpg"
echo "[quest-cam] Trust the :${CAM_PORT} certificate once in the Quest browser."
if [[ "${OPENARM_QUEST_CAMERA_PREVIEW}" =~ ^(1|true|yes|on)$ ]]; then
  echo "[quest-cam] PC preview: OpenCV window 'video16 (quest cam)' opens automatically"
  echo "[quest-cam]   (same frames as MJPEG; scale=${OPENARM_QUEST_CAMERA_PREVIEW_SCALE})"
  echo "[quest-cam]   disable: OPENARM_QUEST_CAMERA_PREVIEW=0"
else
  echo "[quest-cam] PC preview while bridge runs:"
  echo "[quest-cam]   OPENARM_QUEST_CAMERA_PREVIEW=1 ./run_quest_camera_bridge.sh"
  echo "[quest-cam]   or: ./preview_cam_mjpeg.sh"
fi

cd "${ROOT_DIR}"
PREVIEW_ARGS=(--no-camera-preview)
if [[ "${OPENARM_QUEST_CAMERA_PREVIEW}" =~ ^(1|true|yes|on)$ ]]; then
  PREVIEW_ARGS=(--camera-preview --camera-preview-scale "${OPENARM_QUEST_CAMERA_PREVIEW_SCALE}")
fi
exec python quest_bridge.py \
  --host 0.0.0.0 \
  --http-port "${HTTP_PORT}" \
  --camera "${CAM}" \
  --camera-port "${CAM_PORT}" \
  --camera-width "${OPENARM_QUEST_CAMERA_WIDTH:-640}" \
  --camera-height "${OPENARM_QUEST_CAMERA_HEIGHT:-360}" \
  --camera-fps "${OPENARM_QUEST_CAMERA_FPS:-10}" \
  --camera-jpeg "${OPENARM_QUEST_CAMERA_JPEG:-55}" \
  "${PREVIEW_ARGS[@]}" \
  "$@"
