#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Match Quest human_mimic CAN + gripper defaults (record→replay).
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/openarm_vr_compat_env.sh"

ARM_SIDE="${OPENARM_TELEOP_ARM_SIDE:-right_arm}"
if [ "${ARM_SIDE}" = "left_arm" ]; then
  DEFAULT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN}"
  DEFAULT_RECORD_START_PORT=47921
  DEFAULT_RECORD_PATH="${PROJECT_DIR}/recordings/follower_split/left/real_follower_left.csv"
else
  DEFAULT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN}"
  DEFAULT_RECORD_START_PORT=47920
  DEFAULT_RECORD_PATH="${PROJECT_DIR}/recordings/follower_split/right/real_follower_right.csv"
fi

FOLLOWER_CAN="${OPENARM_FOLLOWER_CAN:-${DEFAULT_FOLLOWER_CAN}}"
RECORD_PATH="${OPENARM_RECORD_PATH:-${DEFAULT_RECORD_PATH}}"
POSITION_DUMP_PATH="${OPENARM_POSITION_DUMP_PATH:-${PROJECT_DIR}/recordings/follower_position_snapshots.txt}"
RECORD_START_PORT="${OPENARM_RECORD_START_PORT:-${DEFAULT_RECORD_START_PORT}}"
HZ="${OPENARM_FOLLOWER_RECORD_HZ:-1000}"
LOG_PATH="${OPENARM_REAL_FOLLOWER_RECORD_LOG:-/tmp/openarm_real_follower_record.log}"

run_sudo() {
  if [ -n "${OPENARM_SUDO_PASSWORD:-}" ]; then
    printf '%s\n' "${OPENARM_SUDO_PASSWORD}" | sudo -S -p "" "$@"
  else
    sudo "$@"
  fi
}

configure_can_interfaces() {
  if [ "${OPENARM_SKIP_CAN_CONFIG:-0}" = "1" ]; then
    return
  fi
  if [ "${OPENARM_AUTO_CONFIG_CAN:-1}" != "1" ]; then
    return
  fi

  local bitrate="${OPENARM_CAN_BITRATE:-1000000}"
  local dbitrate="${OPENARM_CAN_DBITRATE:-5000000}"
  if ! ip link show "${FOLLOWER_CAN}" >/dev/null 2>&1; then
    echo "[openarm][ERROR] SocketCAN interface '${FOLLOWER_CAN}' was not found or is not accessible." >&2
    echo "[openarm][HINT] Check interface names with: ip link show type can" >&2
    exit 1
  fi
  echo "[openarm] configuring ${FOLLOWER_CAN} as CAN-FD (${bitrate}/${dbitrate})"
  run_sudo ip link set "${FOLLOWER_CAN}" down
  run_sudo ip link set "${FOLLOWER_CAN}" type can bitrate "${bitrate}" dbitrate "${dbitrate}" fd on
  run_sudo ip link set "${FOLLOWER_CAN}" up
}

mkdir -p "$(dirname "${RECORD_PATH}")"
mkdir -p "$(dirname "${POSITION_DUMP_PATH}")"

FOLLOWER_BIN="${PROJECT_DIR}/build/follower_record_control"
if [ ! -x "${FOLLOWER_BIN}" ]; then
  cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build"
  cmake --build "${PROJECT_DIR}/build" -j2
fi

configure_can_interfaces

echo "[openarm] mode: real hardware follower-only CSV record"
echo "[openarm] arm_side=${ARM_SIDE} follower_can=${FOLLOWER_CAN}"
echo "[openarm] gripper compat: open=${OPENARM_GRIPPER_OPEN} closed=${OPENARM_GRIPPER_CLOSED} Kp=${OPENARM_GRIPPER_KP} Kd=${OPENARM_GRIPPER_KD}"
echo "[openarm] record: ${RECORD_PATH}"
echo "[openarm] position snapshots: ${POSITION_DUMP_PATH}"
echo "[openarm] record_start_port: ${RECORD_START_PORT}"
echo "[openarm] sample rate: ${HZ} Hz"
echo "[openarm] log: ${LOG_PATH}"

unset OPENARM_SUDO_PASSWORD

"${FOLLOWER_BIN}" \
  --arm-side "${ARM_SIDE}" \
  --follower-can "${FOLLOWER_CAN}" \
  --record "${RECORD_PATH}" \
  --record-start-port "${RECORD_START_PORT}" \
  --position-dump "${POSITION_DUMP_PATH}" \
  --hz "${HZ}" \
  "$@" >"${LOG_PATH}" 2>&1 &
FOLLOWER_RECORD_PID=$!

cleanup() {
  if kill -0 "${FOLLOWER_RECORD_PID}" >/dev/null 2>&1; then
    kill "${FOLLOWER_RECORD_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

sleep 1
if ! kill -0 "${FOLLOWER_RECORD_PID}" >/dev/null 2>&1; then
  echo "[openarm][ERROR] follower recorder exited early. Log: ${LOG_PATH}" >&2
  tail -n 80 "${LOG_PATH}" >&2 || true
  exit 1
fi

if [ "${OPENARM_NO_RECORD_PANEL:-0}" = "1" ]; then
  echo "[openarm] record panel disabled; waiting for external start trigger on ${RECORD_START_PORT}"
  wait "${FOLLOWER_RECORD_PID}"
else
  python "${SCRIPT_DIR}/record_start_panel.py" \
    --port "${RECORD_START_PORT}" \
    --log "${LOG_PATH}" \
    --position-output "${POSITION_DUMP_PATH}" \
    --arm-side "${ARM_SIDE}"
fi
