#!/usr/bin/env bash
# Smoothly move both followers from the current pose to a follower_position_snapshots.txt target.
#
# Example:
#   OPENARM_RIGHT_FOLLOWER_CAN=can0 OPENARM_LEFT_FOLLOWER_CAN=can1 \
#     bash script/run_cpp_real_goto_pose.sh \
#     recordings/sessions/taiqi_action1/follower_position_snapshots.txt
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${PROJECT_DIR}/.." && pwd)"

POSE_FILE="${1:-${OPENARM_GOTO_POSE:-}}"
if [ -z "${POSE_FILE}" ]; then
  echo "Usage: $0 PATH/to/follower_position_snapshots.txt" >&2
  exit 2
fi
if [ "$#" -gt 0 ]; then
  shift
fi
if [[ "${POSE_FILE}" != /* ]]; then
  POSE_FILE="${PROJECT_DIR}/${POSE_FILE}"
fi
if [ ! -f "${POSE_FILE}" ]; then
  echo "[openarm][ERROR] pose file not found: ${POSE_FILE}" >&2
  exit 1
fi

URDF="${OPENARM_TELEOP_URDF:-${WORKSPACE_DIR}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
RIGHT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-${OPENARM_FOLLOWER_CAN_RIGHT:-can0}}"
LEFT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-${OPENARM_FOLLOWER_CAN_LEFT:-can1}}"
GOTO_HOLD_SEC="${OPENARM_GOTO_HOLD_SEC:-0}"
GOTO_SPEED="${OPENARM_GOTO_SPEED:-0.18}"
GOTO_MIN_DURATION="${OPENARM_GOTO_MIN_DURATION:-6.0}"
GOTO_SETTLE_TOL="${OPENARM_GOTO_SETTLE_TOL:-0.008}"
GOTO_SETTLE_TIP_M="${OPENARM_GOTO_SETTLE_TIP_M:-0.012}"
GOTO_SETTLE_TIMEOUT="${OPENARM_GOTO_SETTLE_TIMEOUT:-8.0}"
GOTO_SETTLE_KP="${OPENARM_GOTO_SETTLE_KP:-1.5}"
INIT_HOLD="${OPENARM_REPLAY_INIT_HOLD_DURATION:-0.5}"
GAIN_RAMP="${OPENARM_REPLAY_STARTUP_GAIN_RAMP_DURATION:-1.0}"
GAIN_START="${OPENARM_REPLAY_STARTUP_GAIN_RAMP_START_SCALE:-0.1}"

run_sudo() {
  if [ -n "${OPENARM_SUDO_PASSWORD:-}" ]; then
    printf '%s\n' "${OPENARM_SUDO_PASSWORD}" | sudo -S -p "" "$@"
  else
    sudo "$@"
  fi
}

configure_one_can() {
  local iface="$1"
  local bitrate="${OPENARM_CAN_BITRATE:-1000000}"
  local dbitrate="${OPENARM_CAN_DBITRATE:-5000000}"
  if ! ip link show "${iface}" >/dev/null 2>&1; then
    echo "[openarm][ERROR] SocketCAN '${iface}' not found" >&2
    exit 1
  fi
  echo "[openarm] configuring ${iface} as CAN-FD (${bitrate}/${dbitrate})"
  run_sudo ip link set "${iface}" down
  run_sudo ip link set "${iface}" type can bitrate "${bitrate}" dbitrate "${dbitrate}" fd on
  run_sudo ip link set "${iface}" up
}

cd "${PROJECT_DIR}"

BIN="${PROJECT_DIR}/build/dual_replay_control"
if [ ! -x "${BIN}" ]; then
  cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build"
  cmake --build "${PROJECT_DIR}/build" --target dual_replay_control -j"$(nproc 2>/dev/null || echo 2)"
fi

if [ "${OPENARM_SKIP_CAN_CONFIG:-0}" != "1" ] && [ "${OPENARM_AUTO_CONFIG_CAN:-1}" = "1" ]; then
  echo "[openarm] configuring CAN before connecting motors (sudo may ask for password)..."
  configure_one_can "${RIGHT_FOLLOWER_CAN}"
  configure_one_can "${LEFT_FOLLOWER_CAN}"
  echo "[openarm] CAN ready"
fi
unset OPENARM_SUDO_PASSWORD

echo "[openarm] goto-pose target: ${POSE_FILE}"
echo "[openarm] left=${LEFT_FOLLOWER_CAN} right=${RIGHT_FOLLOWER_CAN}"
echo "[openarm] hold=${GOTO_HOLD_SEC}s speed=${GOTO_SPEED} rad/s min_duration=${GOTO_MIN_DURATION}s"
echo "[openarm] settle: joint_tol=${GOTO_SETTLE_TOL} rad tip_tol=${GOTO_SETTLE_TIP_M} m kp×${GOTO_SETTLE_KP}"

exec "${BIN}" \
  --urdf "${URDF}" \
  --goto-pose "${POSE_FILE}" \
  --left-follower-can "${LEFT_FOLLOWER_CAN}" \
  --right-follower-can "${RIGHT_FOLLOWER_CAN}" \
  --goto-hold-sec "${GOTO_HOLD_SEC}" \
  --goto-speed "${GOTO_SPEED}" \
  --goto-min-duration "${GOTO_MIN_DURATION}" \
  --goto-settle-tol "${GOTO_SETTLE_TOL}" \
  --goto-settle-tip-m "${GOTO_SETTLE_TIP_M}" \
  --goto-settle-timeout "${GOTO_SETTLE_TIMEOUT}" \
  --goto-settle-kp-scale "${GOTO_SETTLE_KP}" \
  --init-hold-duration "${INIT_HOLD}" \
  --startup-gain-ramp-duration "${GAIN_RAMP}" \
  --startup-gain-ramp-start-scale "${GAIN_START}" \
  "$@"
