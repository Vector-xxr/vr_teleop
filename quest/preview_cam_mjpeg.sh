#!/usr/bin/env bash
# Preview the Quest camera MJPEG stream on the PC display.
# Use this while run_quest_camera_bridge.sh is running — /dev/video16 is
# already opened by camera_mjpeg_https.py and cannot be shared with ffplay.
set -euo pipefail

CAM_PORT="${OPENARM_QUEST_CAMERA_PORT:-8444}"
HOST="${OPENARM_QUEST_CAMERA_HOST:-127.0.0.1}"
URL="https://${HOST}:${CAM_PORT}/cam.mjpg"

echo "[preview-cam] MJPEG: ${URL}"
echo "[preview-cam] (Self-signed cert — tls_verify disabled for local ffplay.)"
echo "[preview-cam] To open /dev/video16 directly, stop the camera bridge first."

exec ffplay -fflags nobuffer -flags low_delay -tls_verify 0 \
  -window_title "video16 (MJPEG :${CAM_PORT})" \
  "${URL}"
