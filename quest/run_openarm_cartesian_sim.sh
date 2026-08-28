#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TELEOP_ROOT="${REPO_ROOT}/openarm_teleop"
INPLACE_DIR="${TELEOP_ROOT}/openarm_inplace"
QUEST_DIR="${ROOT_DIR}"
ENV_NAME="${OPENARM_QUEST_CONDA_ENV:-openarm_quest}"

XML="${OPENARM_MUJOCO_XML:-${TELEOP_ROOT}/openarm_mujoco/v1/scene_table.xml}"
URDF="${OPENARM_TELEOP_URDF:-${TELEOP_ROOT}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
ARM_SIDE="${OPENARM_ARM_SIDE:-right_arm}"
QUEST_PORT="${OPENARM_QUEST_PORT:-47911}"
HTTP_PORT="${OPENARM_QUEST_HTTP_PORT:-8443}"

CONTROL_PID=""
BRIDGE_PID=""
OWNED_BRIDGE=0

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

stop_stale_control() {
  if pgrep -f '[q]uest_cartesian_control' >/dev/null 2>&1; then
    echo "[quest] stopping stale quest_cartesian_control..."
    pkill -TERM -f '[q]uest_cartesian_control' 2>/dev/null || true
    sleep 0.3
    pkill -KILL -f '[q]uest_cartesian_control' 2>/dev/null || true
  fi
}

cleanup() {
  if [ -n "${CONTROL_PID}" ]; then
    kill -TERM "${CONTROL_PID}" 2>/dev/null || true
    wait "${CONTROL_PID}" 2>/dev/null || true
    CONTROL_PID=""
  fi
  if [ "${OWNED_BRIDGE}" -eq 1 ] && [ -n "${BRIDGE_PID}" ]; then
    kill -TERM "${BRIDGE_PID}" 2>/dev/null || true
    wait "${BRIDGE_PID}" 2>/dev/null || true
    BRIDGE_PID=""
  fi
}
trap cleanup EXIT INT TERM

if [ ! -f "${URDF}" ]; then
  echo "[ERROR] URDF not found: ${URDF}" >&2
  exit 1
fi
if [ ! -f "${XML}" ]; then
  echo "[ERROR] MuJoCo XML not found: ${XML}" >&2
  exit 1
fi

if command -v conda >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source "$(conda info --base)/etc/profile.d/conda.sh"
elif [[ -x "${HOME}/miniconda3/bin/conda" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/miniconda3/etc/profile.d/conda.sh"
else
  echo "conda not found. Run ${QUEST_DIR}/setup_venv.sh first."
  exit 1
fi

if ! conda env list | awk '{print $1}' | grep -qx "${ENV_NAME}"; then
  echo "Conda env '${ENV_NAME}' not found. Run:"
  echo "  ${QUEST_DIR}/setup_venv.sh"
  exit 1
fi

conda activate "${ENV_NAME}"

stop_stale_control

echo "[1/3] Building quest_cartesian_control..."
cmake -S "${INPLACE_DIR}" -B "${INPLACE_DIR}/build"
cmake --build "${INPLACE_DIR}/build" --target quest_cartesian_control -j"$(nproc)"

if port_in_use "${HTTP_PORT}"; then
  echo "[2/3] Quest bridge already listening on :${HTTP_PORT}, skipping start"
  echo "      (stop with: pkill -f quest_bridge.py)"
else
  echo "[2/3] Launching Quest bridge..."
  python "${QUEST_DIR}/quest_bridge.py" --host 0.0.0.0 --http-port "${HTTP_PORT}" &
  BRIDGE_PID=$!
  OWNED_BRIDGE=1
  sleep 1
fi

echo "[3/3] Launching MuJoCo Quest Cartesian sim..."
echo "  URDF=${URDF}"
echo "  XML=${XML}"
echo "  Stop: Ctrl+C in this terminal, or close the MuJoCo window"
if [[ -z "${OPENARM_HM_LOG_DIR:-}" ]]; then
  OPENARM_HM_LOG_DIR="${INPLACE_DIR}/recordings/hm_logs/quest_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${OPENARM_HM_LOG_DIR}"
export OPENARM_HM_LOG_DIR
echo "[quest] HM logs -> ${OPENARM_HM_LOG_DIR}"
cd "${INPLACE_DIR}"
# Extra CLI args pass-through, e.g. OPENARM_QUEST_EXTRA_ARGS="--teleop-mode human_mimic"
read -r -a EXTRA_ARGS <<< "${OPENARM_QUEST_EXTRA_ARGS:-}"
./build/quest_cartesian_control \
  --sim \
  --xml "${XML}" \
  --urdf "${URDF}" \
  --arm-side "${ARM_SIDE}" \
  --quest-port "${QUEST_PORT}" \
  "${EXTRA_ARGS[@]}" &
CONTROL_PID=$!
wait "${CONTROL_PID}"
