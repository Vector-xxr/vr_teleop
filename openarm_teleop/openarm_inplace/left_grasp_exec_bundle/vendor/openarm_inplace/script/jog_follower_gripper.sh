#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

FOLLOWER_CAN="${OPENARM_FOLLOWER_CAN:-follower_right}"
DELTA="${OPENARM_GRIPPER_JOG_DELTA:-0.08}"
STEP_DURATION="${OPENARM_GRIPPER_JOG_STEP_DURATION:-0.8}"
SETTLE_DURATION="${OPENARM_GRIPPER_JOG_SETTLE_DURATION:-0.3}"
KP="${OPENARM_GRIPPER_JOG_KP:-8.0}"
KD="${OPENARM_GRIPPER_JOG_KD:-0.3}"

configure_can_interface() {
  if [ "${OPENARM_AUTO_CONFIG_CAN:-1}" != "1" ]; then
    return
  fi
  local iface="$1"
  local bitrate="${OPENARM_CAN_BITRATE:-1000000}"
  local dbitrate="${OPENARM_CAN_DBITRATE:-5000000}"

  if ! ip link show "${iface}" >/dev/null 2>&1; then
    echo "[openarm][ERROR] SocketCAN interface '${iface}' was not found." >&2
    echo "[openarm][HINT] Check with: ip link show type can" >&2
    exit 1
  fi

  echo "[openarm] configuring ${iface} as CAN-FD (${bitrate}/${dbitrate})"
  sudo ip link set "${iface}" down
  sudo ip link set "${iface}" type can bitrate "${bitrate}" dbitrate "${dbitrate}" fd on
  sudo ip link set "${iface}" up
}

if [ ! -x "${PROJECT_DIR}/build/gripper_jog" ]; then
  cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build"
  cmake --build "${PROJECT_DIR}/build" --target gripper_jog -j2
fi

configure_can_interface "${FOLLOWER_CAN}"

echo "[openarm] jogging follower gripper only"
echo "[openarm] follower_can=${FOLLOWER_CAN} delta=${DELTA} kp=${KP} kd=${KD}"

exec "${PROJECT_DIR}/build/gripper_jog" \
  --can "${FOLLOWER_CAN}" \
  --delta "${DELTA}" \
  --step-duration "${STEP_DURATION}" \
  --settle-duration "${SETTLE_DURATION}" \
  --kp "${KP}" \
  --kd "${KD}" \
  "$@"
