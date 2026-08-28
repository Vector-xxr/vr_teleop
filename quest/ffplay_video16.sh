#!/usr/bin/env bash
# Local preview of Orbbec color camera (default /dev/video16).
# If the device is busy (quest camera bridge), falls back to MJPEG preview.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAM="${OPENARM_QUEST_CAMERA:-16}"
DEV="/dev/video${CAM}"

if [[ ! -e "${DEV}" ]]; then
  echo "[ffplay-video16][ERROR] ${DEV} not found" >&2
  exit 1
fi

if command -v fuser >/dev/null 2>&1 && fuser "${DEV}" >/dev/null 2>&1; then
  echo "[ffplay-video16] ${DEV} is busy (quest camera bridge holds exclusive V4L2 access):"
  fuser -v "${DEV}" 2>&1 || true
  echo "[ffplay-video16] → using MJPEG preview instead (same camera, no second open)."
  exec "${ROOT_DIR}/preview_cam_mjpeg.sh"
fi

echo "[ffplay-video16] opening ${DEV} directly (1280x720 MJPEG mode)"
exec ffplay -f v4l2 -input_format mjpeg -video_size 1280x720 -framerate 30 \
  -i "${DEV}" -window_title "video16 Orbbec"
