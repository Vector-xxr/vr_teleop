#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TELEOP_ROOT="$(cd "${PROJECT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${TELEOP_ROOT}/.." && pwd)"
QUEST_DIR="${REPO_ROOT}/quest"

XML="${OPENARM_MUJOCO_XML:-${TELEOP_ROOT}/openarm_mujoco/v1/scene_table.xml}"
URDF="${OPENARM_TELEOP_URDF:-${TELEOP_ROOT}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
ARM_SIDE="${OPENARM_ARM_SIDE:-right_arm}"
QUEST_PORT="${OPENARM_QUEST_PORT:-47911}"
HTTP_PORT="${OPENARM_QUEST_HTTP_PORT:-8443}"
ENV_NAME="${OPENARM_QUEST_CONDA_ENV:-openarm_quest}"
START_BRIDGE="${OPENARM_START_QUEST_BRIDGE:-1}"

port_in_use() {
  local port="$1"
  if command -v ss >/dev/null 2>&1; then
    ss -ltn 2>/dev/null | grep -q ":${port} "
    return $?
  fi
  if command -v lsof >/dev/null 2>&1; then
    lsof -iTCP:"${port}" -sTCP:LISTEN >/dev/null 2>&1
    return $?
  fi
  return 1
}

if [ ! -f "${URDF}" ]; then
  echo "[openarm][ERROR] URDF not found: ${URDF}" >&2
  exit 1
fi
if [ ! -f "${XML}" ]; then
  echo "[openarm][ERROR] MuJoCo XML not found: ${XML}" >&2
  exit 1
fi

if [ ! -x "${PROJECT_DIR}/build/quest_cartesian_control" ]; then
  cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build"
  cmake --build "${PROJECT_DIR}/build" --target quest_cartesian_control -j"$(nproc)"
fi

BRIDGE_PID=""
cleanup() {
  if [ -n "${BRIDGE_PID}" ]; then
    kill "${BRIDGE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [ "${START_BRIDGE}" = "1" ]; then
  if port_in_use "${HTTP_PORT}"; then
    echo "[openarm] Quest bridge already listening on :${HTTP_PORT}, skipping start"
  else
    if command -v conda >/dev/null 2>&1; then
      # shellcheck disable=SC1091
      source "$(conda info --base)/etc/profile.d/conda.sh"
      conda activate "${ENV_NAME}"
    fi
    echo "[openarm] starting Quest bridge on HTTP ${HTTP_PORT}, UDP ${QUEST_PORT}"
    python "${QUEST_DIR}/quest_bridge.py" --host 0.0.0.0 --http-port "${HTTP_PORT}" &
    BRIDGE_PID=$!
    sleep 1
  fi
fi

echo "[openarm] MuJoCo Quest Cartesian sim"
echo "[openarm] XML=${XML}"
echo "[openarm] URDF=${URDF}"
echo "[openarm] arm_side=${ARM_SIDE} quest_port=${QUEST_PORT}"

cd "${PROJECT_DIR}"
exec ./build/quest_cartesian_control \
  --sim \
  --xml "${XML}" \
  --urdf "${URDF}" \
  --arm-side "${ARM_SIDE}" \
  --quest-port "${QUEST_PORT}"
