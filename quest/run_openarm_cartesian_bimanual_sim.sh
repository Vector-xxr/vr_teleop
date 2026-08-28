#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${ROOT_DIR}/.." && pwd)"
TELEOP_ROOT="${REPO_ROOT}/openarm_teleop"
INPLACE_DIR="${TELEOP_ROOT}/openarm_inplace"
QUEST_DIR="${ROOT_DIR}"
ENV_NAME="${OPENARM_QUEST_CONDA_ENV:-openarm_quest}"

XML="${OPENARM_MUJOCO_XML:-${TELEOP_ROOT}/openarm_mujoco/v1/scene_table.xml}"
URDF="${OPENARM_TELEOP_URDF:-${TELEOP_ROOT}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
QUEST_PORT_LEFT="${OPENARM_QUEST_PORT_LEFT:-47912}"
QUEST_PORT_RIGHT="${OPENARM_QUEST_PORT_RIGHT:-47911}"
HTTP_PORT="${OPENARM_QUEST_HTTP_PORT:-8443}"

CONTROL_PID=""
BRIDGE_PID=""
RECORD_PANEL_PID=""
JOG_PANEL_PID=""
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

stop_stale_bridge() {
  if pgrep -f '[q]uest_bridge.py' >/dev/null 2>&1; then
    echo "[quest] stopping stale quest_bridge.py (reload client.html + button UDP)..."
    pkill -TERM -f '[q]uest_bridge.py' 2>/dev/null || true
    sleep 0.3
    pkill -KILL -f '[q]uest_bridge.py' 2>/dev/null || true
  fi
}

cleanup() {
  if [ -n "${RECORD_PANEL_PID}" ]; then
    kill -TERM "${RECORD_PANEL_PID}" 2>/dev/null || true
    wait "${RECORD_PANEL_PID}" 2>/dev/null || true
    RECORD_PANEL_PID=""
  fi
  if [ -n "${JOG_PANEL_PID}" ]; then
    kill -TERM "${JOG_PANEL_PID}" 2>/dev/null || true
    wait "${JOG_PANEL_PID}" 2>/dev/null || true
    JOG_PANEL_PID=""
  fi
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
stop_stale_bridge

echo "[1/3] Building quest_cartesian_control..."
cmake -S "${INPLACE_DIR}" -B "${INPLACE_DIR}/build"
cmake --build "${INPLACE_DIR}/build" --target quest_cartesian_control -j"$(nproc)"

echo "[2/3] Launching Quest bridge (bimanual)..."
python "${QUEST_DIR}/quest_bridge.py" \
  --host 0.0.0.0 \
  --http-port "${HTTP_PORT}" \
  --udp-left-port "${QUEST_PORT_LEFT}" \
  --udp-right-port "${QUEST_PORT_RIGHT}" &
BRIDGE_PID=$!
OWNED_BRIDGE=1
sleep 1

echo "[3/3] Launching MuJoCo Quest Bimanual sim..."
echo "  URDF=${URDF}"
echo "  XML=${XML}"
echo "  Left UDP=${QUEST_PORT_LEFT}  Right UDP=${QUEST_PORT_RIGHT}"
echo "  Stop: Ctrl+C in this terminal, or close the MuJoCo window"
if [[ -z "${OPENARM_HM_LOG_DIR:-}" ]]; then
  OPENARM_HM_LOG_DIR="${INPLACE_DIR}/recordings/hm_logs/quest_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${OPENARM_HM_LOG_DIR}"
export OPENARM_HM_LOG_DIR
echo "[quest] HM logs -> ${OPENARM_HM_LOG_DIR}"
cd "${INPLACE_DIR}"
./build/quest_cartesian_control \
  --sim \
  --bimanual \
  --xml "${XML}" \
  --urdf "${URDF}" \
  --quest-port-left "${QUEST_PORT_LEFT}" \
  --quest-port-right "${QUEST_PORT_RIGHT}" \
  ${OPENARM_QUEST_EXTRA_ARGS:-} &
CONTROL_PID=$!
sleep 1
if [[ "${OPENARM_QUEST_UI:-1}" == "1" ]]; then
  jog_speed_mms="$(python3 -c 'import os; print(float(os.environ.get("OPENARM_QUEST_JOG_SPEED","0.02"))*1000.0)')"
  echo "[quest] Launching action-record + TCP-jog panels (hold-to-jog ${jog_speed_mms} mm/s)..."
  python3 "${QUEST_DIR}/quest_action_record_panel.py" --port "${OPENARM_QUEST_UI_RECORD_PORT:-47930}" &
  RECORD_PANEL_PID=$!
  python3 "${QUEST_DIR}/quest_tcp_jog_panel.py" --port "${OPENARM_QUEST_UI_JOG_PORT:-47931}" --speed "${jog_speed_mms}" &
  JOG_PANEL_PID=$!
fi
wait "${CONTROL_PID}"
