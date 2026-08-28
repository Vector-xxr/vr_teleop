#!/usr/bin/env bash
# Phase-1 body keypoints bridge for BODY_SKELETON_RETARGET (UDP 47952).
# Does not control the robot. Pair with:
#   OPENARM_QUEST_EXTRA_ARGS="--body-pose" ./quest/run_openarm_cartesian_bimanual_sim.sh
#
# Env:
#   OPENARM_BODY_POSE_HOST / OPENARM_BODY_POSE_PORT
#   OPENARM_HYBRIK_CAM / OPENARM_HYBRIK_SHOW
#   OPENARM_BODY_POSE_BACKEND  hybrik (default) | mediapipe
#   QT_QPA_PLATFORM            default xcb when show=1
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
HYBRIK_DIR="$REPO_ROOT/HybrIK"

HOST="${OPENARM_BODY_POSE_HOST:-127.0.0.1}"
PORT="${OPENARM_BODY_POSE_PORT:-47952}"
CAM="${OPENARM_HYBRIK_CAM:-8}"
SHOW="${OPENARM_HYBRIK_SHOW:-1}"
BACKEND="${OPENARM_BODY_POSE_BACKEND:-hybrik}"
CONDA_ENV="${CONDA_ENV:-hybrik}"
SMOOTH="${OPENARM_HYBRIK_SMOOTH:-0.28}"

if [[ "${CONDA_DEFAULT_ENV:-}" != "$CONDA_ENV" ]]; then
  if command -v conda >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source "$(conda info --base)/etc/profile.d/conda.sh"
    conda activate "$CONDA_ENV"
  else
    echo "[run_body_pose_bridge] warning: conda not found; using current python" >&2
  fi
fi

if [[ "$SHOW" == "1" ]]; then
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
fi

SHOW_FLAG=()
if [[ "$SHOW" == "1" ]]; then
  SHOW_FLAG=(--show)
fi

echo "[run_body_pose_bridge] body keypoints -> udp://${HOST}:${PORT}  cam=${CAM}  backend=${BACKEND}"

# HybrIK models live under HybrIK/; script still runs from quest/ but adapter chdirs.
cd "$SCRIPT_DIR"
exec python -u body_pose_bridge.py \
  --backend "$BACKEND" \
  --cam "$CAM" \
  --udp-host "$HOST" \
  --udp-port "$PORT" \
  --fast \
  --no-flip \
  --no-set-res \
  --smooth "$SMOOTH" \
  "${SHOW_FLAG[@]}" \
  "$@"
