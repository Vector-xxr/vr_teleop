#!/usr/bin/env bash
# Start persistent job service:
#   startup once -> idle -> on command run recordings/jobN/action1 -> 2D grasp -> action2
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKEWER_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VR_CONTROL_DIR="$(cd "${SKEWER_DIR}/.." && pwd)"

# Default: openarm_inplace/recordings  (contains job1/, job2/, tasks/, ...)
DEFAULT_JOBS_ROOT="${VR_CONTROL_DIR}/openarm_teleop/openarm_inplace/recordings"
JOBS_ROOT="${OPENARM_SKEWER_JOBS_ROOT:-${DEFAULT_JOBS_ROOT}}"
RUNTIME_ROOT="${OPENARM_SKEWER_RUNTIME_ROOT:-${SKEWER_DIR}/jobs/_runtime}"
JOB_LISTEN_PORT="${OPENARM_SKEWER_JOB_LISTEN_PORT:-47925}"
COMMAND_PORT="${OPENARM_QUEST_ALIGN_COMMAND_PORT:-47920}"
STATUS_PORT="${OPENARM_QUEST_ALIGN_STATUS_PORT:-47921}"
# Field CAN names (same as run_cpp_real_replay_both.sh / VR live pipeline)
LEFT_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-${OPENARM_FOLLOWER_CAN_LEFT:-follower_left}}"
RIGHT_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-${OPENARM_FOLLOWER_CAN_RIGHT:-follower_right}}"
CAMERA_BACKEND="${OPENARM_RIGHT_CAMERA_BACKEND:-opencv}"
CAMERA_INDEX="${OPENARM_RIGHT_CAMERA_INDEX:-16}"

ARGS=(
  --jobs-root "${JOBS_ROOT}"
  --runtime-root "${RUNTIME_ROOT}"
  --job-listen-port "${JOB_LISTEN_PORT}"
  --command-port "${COMMAND_PORT}"
  --status-port "${STATUS_PORT}"
  --left-can "${LEFT_CAN}"
  --right-can "${RIGHT_CAN}"
  --camera-backend "${CAMERA_BACKEND}"
  --camera-index "${CAMERA_INDEX}"
)

if [ -n "${OPENARM_SUDO_PASSWORD:-}" ]; then
  ARGS+=(--sudo-password "${OPENARM_SUDO_PASSWORD}")
fi
if [ -n "${OPENARM_GRASP_DESCENT_M:-}" ]; then
  ARGS+=(--grasp-descent-m "${OPENARM_GRASP_DESCENT_M}")
fi

echo "[skewer-job] jobs-root=${JOBS_ROOT}"
echo "[skewer-job] CAN: left=${LEFT_CAN} right=${RIGHT_CAN}"
echo "[skewer-job] expect: ${JOBS_ROOT}/job1/{action1,action2}.csv"
echo "[skewer-job] expect: ${JOBS_ROOT}/job2/{action1,action2}.csv"

cd "${SKEWER_DIR}"
exec python3 scripts/skewer_job_service.py "${ARGS[@]}" "$@"
