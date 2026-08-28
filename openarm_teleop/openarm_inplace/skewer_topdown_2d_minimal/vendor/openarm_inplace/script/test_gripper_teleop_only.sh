#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${PROJECT_DIR}/.." && pwd)"

URDF="${OPENARM_TELEOP_URDF:-${WORKSPACE_DIR}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
ARM_SIDE="${OPENARM_TELEOP_ARM_SIDE:-right_arm}"
if [ "${1:-}" = "left" ] || [ "${1:-}" = "left_arm" ]; then
  ARM_SIDE="left_arm"
  shift
elif [ "${1:-}" = "right" ] || [ "${1:-}" = "right_arm" ]; then
  ARM_SIDE="right_arm"
  shift
fi

if [ "${ARM_SIDE}" = "left_arm" ]; then
  DEFAULT_LEADER_CAN="leader_left"
  DEFAULT_FOLLOWER_CAN="follower_left"
else
  DEFAULT_LEADER_CAN="leader_right"
  DEFAULT_FOLLOWER_CAN="follower_right"
fi

LEADER_CAN="${OPENARM_LEADER_CAN:-${DEFAULT_LEADER_CAN}}"
FOLLOWER_CAN="${OPENARM_FOLLOWER_CAN:-${DEFAULT_FOLLOWER_CAN}}"
HZ="${OPENARM_GRIPPER_TEST_HZ:-1000}"
PREZERO_DURATION="${OPENARM_GRIPPER_TEST_PREZERO_DURATION:-2.0}"
PREZERO_TARGET="${OPENARM_GRIPPER_TEST_PREZERO_TARGET:-0.0}"

configure_can_interfaces() {
  if [ "${OPENARM_AUTO_CONFIG_CAN:-1}" != "1" ]; then
    return
  fi
  local bitrate="${OPENARM_CAN_BITRATE:-1000000}"
  local dbitrate="${OPENARM_CAN_DBITRATE:-5000000}"
  local configured=" "
  for iface in "$@"; do
    if [[ "${configured}" == *" ${iface} "* ]]; then
      continue
    fi
    configured="${configured}${iface} "
    if ! ip link show "${iface}" >/dev/null 2>&1; then
      echo "[openarm][ERROR] SocketCAN interface '${iface}' was not found." >&2
      echo "[openarm][HINT] Check with: ip link show type can" >&2
      exit 1
    fi
    echo "[openarm] configuring ${iface} as CAN-FD (${bitrate}/${dbitrate})"
    sudo ip link set "${iface}" down
    sudo ip link set "${iface}" type can bitrate "${bitrate}" dbitrate "${dbitrate}" fd on
    sudo ip link set "${iface}" up
  done
}

BIN="${PROJECT_DIR}/build/gripper_teleop_test"
if [ ! -x "${BIN}" ]; then
  cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build"
  cmake --build "${PROJECT_DIR}/build" --target gripper_teleop_test -j2
fi

configure_can_interfaces "${LEADER_CAN}" "${FOLLOWER_CAN}"

echo "[openarm] gripper-only leader/follower test"
echo "[openarm] URDF: ${URDF}"
echo "[openarm] arm_side=${ARM_SIDE} leader_can=${LEADER_CAN} follower_can=${FOLLOWER_CAN}"
echo "[openarm] full Control::unilateral_step path; follower arm joints hold current position"
echo "[openarm] prezero follower gripper: target=${PREZERO_TARGET} duration=${PREZERO_DURATION}s"
echo "[openarm] target = leader gripper q, with dq=0 and tau=0 like current teleop command path"

cd "${PROJECT_DIR}"
exec "${BIN}" "${URDF}" \
  --arm-side "${ARM_SIDE}" \
  --leader-can "${LEADER_CAN}" \
  --follower-can "${FOLLOWER_CAN}" \
  --hz "${HZ}" \
  --prezero-duration "${PREZERO_DURATION}" \
  --prezero-target "${PREZERO_TARGET}" \
  "$@"
