#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${PROJECT_DIR}/.." && pwd)"

CSV="${1:-${OPENARM_REPLAY_PATH:-}}"
if [ -z "${CSV}" ]; then
  echo "Usage: $0 RECORDING.csv_or_dir" >&2
  echo "Or set OPENARM_REPLAY_PATH=recordings/file.csv" >&2
  exit 2
fi
if [ "$#" -gt 0 ]; then
  shift
fi

if [[ "${CSV}" != /* ]]; then
  CSV="${PROJECT_DIR}/${CSV}"
fi

XML="${OPENARM_MUJOCO_XML:-${WORKSPACE_DIR}/openarm_mujoco/v1/scene.xml}"
URDF="${OPENARM_TELEOP_URDF:-${WORKSPACE_DIR}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
SIDE="${OPENARM_TELEOP_SIDE:-right}"

if [ ! -f "${CSV}" ] && [ ! -d "${CSV}" ]; then
  echo "[openarm_inplace] replay path not found: ${CSV}" >&2
  exit 1
fi

if [ -f "${CSV}" ] && [ ! -s "${CSV}" ]; then
  echo "[openarm_inplace] replay CSV is empty: ${CSV}" >&2
  exit 1
fi

if [ ! -x "${PROJECT_DIR}/build/mujoco_panel_control" ]; then
  cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build"
  cmake --build "${PROJECT_DIR}/build" -j2
fi

echo "[openarm_inplace] C++ MuJoCo replay"
echo "[openarm_inplace] project: ${PROJECT_DIR}"
echo "[openarm_inplace] XML: ${XML}"
echo "[openarm_inplace] URDF: ${URDF}"
echo "[openarm_inplace] replay: ${CSV}"

CMD=("${PROJECT_DIR}/build/mujoco_panel_control" \
  --xml "${XML}" \
  --urdf "${URDF}" \
  --side "${SIDE}" \
  --replay "${CSV}")
CMD+=("$@")

exec "${CMD[@]}"
