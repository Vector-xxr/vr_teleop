#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TARGET_BIN="${PROJECT_DIR}/build/gripper_only_teleop_test"

SIDE="${OPENARM_GRIPPER_TELEOP_TEST_SIDE:-both}"
RIGHT_LEADER_CAN="${OPENARM_RIGHT_LEADER_CAN:-leader_right}"
RIGHT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-follower_right}"
LEFT_LEADER_CAN="${OPENARM_LEFT_LEADER_CAN:-leader_left}"
LEFT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-follower_left}"
RIGHT_LOG="${OPENARM_RIGHT_GRIPPER_TELEOP_LOG:-/tmp/openarm_gripper_teleop_right.log}"
LEFT_LOG="${OPENARM_LEFT_GRIPPER_TELEOP_LOG:-/tmp/openarm_gripper_teleop_left.log}"
HZ="${OPENARM_GRIPPER_TELEOP_HZ:-200}"
KP="${OPENARM_GRIPPER_TELEOP_KP:-8.0}"
KD="${OPENARM_GRIPPER_TELEOP_KD:-0.3}"
OFFSET="${OPENARM_GRIPPER_TELEOP_OFFSET:-0.0}"
SEND_ID="${OPENARM_GRIPPER_SEND_CAN_ID:-0x08}"
RECV_ID="${OPENARM_GRIPPER_RECV_CAN_ID:-0x18}"

usage() {
  cat <<EOF
Usage: $0 [--side left|right|both] [--hz HZ] [--kp K] [--kd D]

Leader-follower gripper-only test for real hardware.

Only the gripper motor is initialized/enabled on each selected CAN bus. Arm
motors are not initialized, enabled, held, or commanded.

  leader gripper feedback -> follower gripper MIT position target

Options:
  --side left|right|both   Which arm pair(s) to start. Default: ${SIDE}
  --hz HZ                 Control loop frequency. Default: ${HZ}
  --kp K                  Follower gripper position gain. Default: ${KP}
  --kd D                  Follower gripper damping gain. Default: ${KD}
  --offset RAD            Add position offset to follower target. Default: ${OFFSET}
  --send-id ID            Gripper send CAN ID. Default: ${SEND_ID}
  --recv-id ID            Gripper receive CAN ID. Default: ${RECV_ID}
  -h, --help              Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --side)
      SIDE="$2"
      shift 2
      ;;
    --hz)
      HZ="$2"
      shift 2
      ;;
    --kp)
      KP="$2"
      shift 2
      ;;
    --kd)
      KD="$2"
      shift 2
      ;;
    --offset)
      OFFSET="$2"
      shift 2
      ;;
    --send-id)
      SEND_ID="$2"
      shift 2
      ;;
    --recv-id)
      RECV_ID="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[openarm][ERROR] unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${SIDE}" in
  left|right|both) ;;
  *)
    echo "[openarm][ERROR] --side must be left, right, or both; got '${SIDE}'" >&2
    exit 2
    ;;
esac

configure_can_interfaces() {
  if [[ "${OPENARM_AUTO_CONFIG_CAN:-1}" != "1" ]]; then
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
      echo "[openarm][ERROR] SocketCAN interface '${iface}' was not found or is not accessible." >&2
      echo "[openarm][HINT] Check interface names with: ip link show type can" >&2
      exit 1
    fi

    echo "[openarm] configuring ${iface} as CAN-FD (${bitrate}/${dbitrate})"
    sudo ip link set "${iface}" down
    sudo ip link set "${iface}" type can bitrate "${bitrate}" dbitrate "${dbitrate}" fd on
    sudo ip link set "${iface}" up
  done
}

RIGHT_PID=""
LEFT_PID=""

cleanup() {
  if [[ -n "${RIGHT_PID}" ]] && kill -0 "${RIGHT_PID}" >/dev/null 2>&1; then
    kill "${RIGHT_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${LEFT_PID}" ]] && kill -0 "${LEFT_PID}" >/dev/null 2>&1; then
    kill "${LEFT_PID}" >/dev/null 2>&1 || true
  fi
  wait "${RIGHT_PID:-0}" >/dev/null 2>&1 || true
  wait "${LEFT_PID:-0}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

CAN_INTERFACES=()
if [[ "${SIDE}" == "right" || "${SIDE}" == "both" ]]; then
  CAN_INTERFACES+=("${RIGHT_LEADER_CAN}" "${RIGHT_FOLLOWER_CAN}")
fi
if [[ "${SIDE}" == "left" || "${SIDE}" == "both" ]]; then
  CAN_INTERFACES+=("${LEFT_LEADER_CAN}" "${LEFT_FOLLOWER_CAN}")
fi
configure_can_interfaces "${CAN_INTERFACES[@]}"

if [[ ! -x "${TARGET_BIN}" \
      || "${PROJECT_DIR}/control/openarm_gripper_only_teleop_test.cpp" -nt "${TARGET_BIN}" \
      || "${PROJECT_DIR}/CMakeLists.txt" -nt "${TARGET_BIN}" ]]; then
  cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build"
  cmake --build "${PROJECT_DIR}/build" --target gripper_only_teleop_test -j2
fi

if [[ "${SIDE}" == "right" || "${SIDE}" == "both" ]]; then
  echo "[openarm] starting RIGHT gripper-only master-slave test"
  echo "[openarm] right leader=${RIGHT_LEADER_CAN} follower=${RIGHT_FOLLOWER_CAN}"
  "${TARGET_BIN}" \
    --leader-can "${RIGHT_LEADER_CAN}" \
    --follower-can "${RIGHT_FOLLOWER_CAN}" \
    --hz "${HZ}" --kp "${KP}" --kd "${KD}" --offset "${OFFSET}" \
    --send-id "${SEND_ID}" --recv-id "${RECV_ID}" >"${RIGHT_LOG}" 2>&1 &
  RIGHT_PID=$!
fi

sleep 0.8

if [[ "${SIDE}" == "left" || "${SIDE}" == "both" ]]; then
  echo "[openarm] starting LEFT gripper-only master-slave test"
  echo "[openarm] left leader=${LEFT_LEADER_CAN} follower=${LEFT_FOLLOWER_CAN}"
  "${TARGET_BIN}" \
    --leader-can "${LEFT_LEADER_CAN}" \
    --follower-can "${LEFT_FOLLOWER_CAN}" \
    --hz "${HZ}" --kp "${KP}" --kd "${KD}" --offset "${OFFSET}" \
    --send-id "${SEND_ID}" --recv-id "${RECV_ID}" >"${LEFT_LOG}" 2>&1 &
  LEFT_PID=$!
fi

echo "[openarm] gripper-only master-slave test is running"
echo "[openarm] Only gripper motors were initialized/enabled."
echo "[openarm] Manually move the leader gripper(s) and observe follower gripper tracking."
echo "[openarm] right log: ${RIGHT_LOG}"
echo "[openarm] left log : ${LEFT_LOG}"
echo "[openarm] press Ctrl+C to stop"

while true; do
  if [[ -n "${RIGHT_PID}" ]] && ! kill -0 "${RIGHT_PID}" >/dev/null 2>&1; then
    echo "[openarm][WARN] right gripper test process exited"
    exit 1
  fi
  if [[ -n "${LEFT_PID}" ]] && ! kill -0 "${LEFT_PID}" >/dev/null 2>&1; then
    echo "[openarm][WARN] left gripper test process exited"
    exit 1
  fi
  sleep 1
done
