#!/usr/bin/env bash
# Replay ONLY trajectories in recordings/import_task, then hold final pose.
#
# - Motor / gripper config: same as VR teleop (openarm_vr_compat_env.sh)
# - Right grasp-hold torque: slightly stronger than default VR (this script only)
# - No steady / action1 / vision / other pipelines
# - Does not modify other project programs
#
# Usage (from vr_control root):
#   ./run_import_task.sh
#   ./run_import_task.sh /path/to/import_task
#   OPENARM_IMPORT_TASK_SPEED=0.5 ./run_import_task.sh
#
# Ctrl+C after finish: graceful shutdown (Kd damping release).

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INPLACE_DIR="${ROOT_DIR}/openarm_teleop/openarm_inplace"
SCRIPT_DIR="${INPLACE_DIR}/script"
DEFAULT_TASK_DIR="${INPLACE_DIR}/recordings/import_task"
TASK_DIR="${1:-${OPENARM_IMPORT_TASK_DIR:-${DEFAULT_TASK_DIR}}}"
if [[ "${TASK_DIR}" != /* ]]; then
  TASK_DIR="${ROOT_DIR}/${TASK_DIR}"
fi
TASK_DIR="$(cd "${TASK_DIR}" && pwd)"
if [ "$#" -gt 0 ]; then
  shift
fi

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/openarm_vr_compat_env.sh"

# Right grasp hold a bit stronger than default VR (τ≈Kp·lag; Kp stays 10).
export OPENARM_GRIPPER_GRASP_LAG_RIGHT="${OPENARM_IMPORT_TASK_GRASP_LAG_RIGHT:-0.32}"
export OPENARM_GRIPPER_GRASP_LAG_MAX_RIGHT="${OPENARM_IMPORT_TASK_GRASP_LAG_MAX_RIGHT:-0.40}"

# This pipeline: no extra startup home choreography beyond first-frame blend.
export OPENARM_REPLAY_AUTO_HOME="${OPENARM_REPLAY_AUTO_HOME:-0}"
export OPENARM_TORQUE_PANEL="${OPENARM_TORQUE_PANEL:-0}"
export OPENARM_REPLAY_MOTOR_LOG="${OPENARM_REPLAY_MOTOR_LOG:-off}"

COMMAND_PORT="${OPENARM_REPLAY_COMMAND_PORT:-47970}"
STATUS_PORT="${OPENARM_REPLAY_STATUS_PORT:-$((COMMAND_PORT + 1))}"
SPEED="${OPENARM_IMPORT_TASK_SPEED:-${OPENARM_DUAL_REPLAY_SPEED:-1.0}}"
INITIAL_TRANSITION="${OPENARM_DUAL_REPLAY_INITIAL_TRANSITION_DURATION:-3.0}"
TRANSITION="${OPENARM_DUAL_REPLAY_TRANSITION_DURATION:-5.0}"
READY_TIMEOUT_S="${OPENARM_IMPORT_TASK_READY_TIMEOUT_S:-90}"
PLAY_TIMEOUT_S="${OPENARM_IMPORT_TASK_PLAY_TIMEOUT_S:-7200}"

PREPARE_PY="${DEFAULT_TASK_DIR}/prepare_import_replay.py"
ORCH_PY="${DEFAULT_TASK_DIR}/run_import_task_orchestrator.py"
SERVER_SH="${SCRIPT_DIR}/run_cpp_real_replay_server.sh"
DUAL_BIN="${INPLACE_DIR}/build/dual_replay_control"

if [ ! -x "${DUAL_BIN}" ]; then
  echo "[import-task] building dual_replay_control..."
  cmake -S "${INPLACE_DIR}" -B "${INPLACE_DIR}/build"
  cmake --build "${INPLACE_DIR}/build" --target dual_replay_control -j"$(nproc)"
fi

PREPARE_LOG="$(mktemp /tmp/openarm_import_task_prepare.XXXXXX.log)"
python3 "${PREPARE_PY}" "${TASK_DIR}" | tee "${PREPARE_LOG}"
MERGED_DIR="$(grep '^MERGED_DIR=' "${PREPARE_LOG}" | tail -n 1 | cut -d= -f2-)"
ACTION_COUNT="$(grep '^ACTION_COUNT=' "${PREPARE_LOG}" | tail -n 1 | cut -d= -f2-)"
rm -f "${PREPARE_LOG}"
if [ -z "${MERGED_DIR}" ] || [ ! -d "${MERGED_DIR}" ]; then
  echo "[import-task][ERROR] prepare failed (no MERGED_DIR)" >&2
  exit 1
fi

export OPENARM_DUAL_REPLAY_INITIAL_TRANSITION_DURATION="${INITIAL_TRANSITION}"
export OPENARM_DUAL_REPLAY_TRANSITION_DURATION="${TRANSITION}"
export OPENARM_REPLAY_COMMAND_PORT="${COMMAND_PORT}"
export OPENARM_REPLAY_STATUS_PORT="${STATUS_PORT}"

SERVER_LOG="$(mktemp /tmp/openarm_import_task_server.XXXXXX.log)"

echo "[import-task] task dir     : ${TASK_DIR}"
echo "[import-task] actions      : ${ACTION_COUNT}  merged=${MERGED_DIR}"
echo "[import-task] gripper MIT  : Kp=${OPENARM_GRIPPER_KP} Kd=${OPENARM_GRIPPER_KD}"
echo "[import-task] contact-hold : L=${OPENARM_GRIPPER_CONTACT_HOLD_LEFT} R=${OPENARM_GRIPPER_CONTACT_HOLD_RIGHT}"
echo "[import-task] right lag    : ${OPENARM_GRIPPER_GRASP_LAG_RIGHT} / max ${OPENARM_GRIPPER_GRASP_LAG_MAX_RIGHT} (stronger than default VR)"
echo "[import-task] speed        : ${SPEED}x"
echo "[import-task] auto-home    : ${OPENARM_REPLAY_AUTO_HOME}"

set +e
python3 "${ORCH_PY}" \
  --command-port "${COMMAND_PORT}" \
  --status-port "${STATUS_PORT}" \
  --merged-dir "${MERGED_DIR}" \
  --speed "${SPEED}" \
  --ready-timeout-s "${READY_TIMEOUT_S}" \
  --play-timeout-s "${PLAY_TIMEOUT_S}" \
  --server-cwd "${INPLACE_DIR}" \
  --server-sh "${SERVER_SH}" \
  --server-log "${SERVER_LOG}"
code=$?
set -e
echo "[import-task] server log: ${SERVER_LOG}"
exit "${code}"
