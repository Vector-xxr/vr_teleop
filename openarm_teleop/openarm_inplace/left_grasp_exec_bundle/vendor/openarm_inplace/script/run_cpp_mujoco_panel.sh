#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${PROJECT_DIR}/.." && pwd)"

XML="${OPENARM_MUJOCO_XML:-${WORKSPACE_DIR}/openarm_mujoco/v1/scene.xml}"
URDF="${OPENARM_TELEOP_URDF:-${WORKSPACE_DIR}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
SIDE="${OPENARM_TELEOP_SIDE:-right}"
PANEL_PORT="${OPENARM_PANEL_PORT:-47910}"
RECORD_PATH="${OPENARM_RECORD_PATH:-${PROJECT_DIR}/recordings/mujoco_panel.csv}"
DEBUG_CSV="${OPENARM_DEBUG_CSV:-}"

mkdir -p "$(dirname "${RECORD_PATH}")"
if [ -n "${DEBUG_CSV}" ]; then
  mkdir -p "$(dirname "${DEBUG_CSV}")"
fi

if [ ! -x "${PROJECT_DIR}/build/mujoco_panel_control" ]; then
  cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build"
  cmake --build "${PROJECT_DIR}/build" -j2
fi

echo "[openarm] C++ MuJoCo teleop core"
echo "[openarm] XML: ${XML}"
echo "[openarm] URDF: ${URDF}"
echo "[openarm] side=${SIDE} panel_port=${PANEL_PORT}"
echo "[openarm] record: ${RECORD_PATH}"
if [ -n "${DEBUG_CSV}" ]; then
  echo "[openarm] debug_csv: ${DEBUG_CSV}"
fi

CMD=("${PROJECT_DIR}/build/mujoco_panel_control"
  --xml "${XML}" --urdf "${URDF}" --side "${SIDE}"
  --panel-port "${PANEL_PORT}" --record "${RECORD_PATH}")
if [ -n "${DEBUG_CSV}" ]; then
  CMD+=(--debug-csv "${DEBUG_CSV}")
fi

exec "${CMD[@]}" "$@"
