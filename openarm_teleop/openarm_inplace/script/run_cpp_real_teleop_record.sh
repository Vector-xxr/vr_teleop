#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${PROJECT_DIR}/.." && pwd)"

URDF="${OPENARM_TELEOP_URDF:-${WORKSPACE_DIR}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
ARM_SIDE="${OPENARM_TELEOP_ARM_SIDE:-right_arm}"
if [ "${ARM_SIDE}" = "left_arm" ]; then
  DEFAULT_LEADER_CAN="leader_left"
  DEFAULT_FOLLOWER_CAN="follower_left"
else
  DEFAULT_LEADER_CAN="leader_right"
  DEFAULT_FOLLOWER_CAN="follower_right"
fi
LEADER_CAN="${OPENARM_LEADER_CAN:-${DEFAULT_LEADER_CAN}}"
FOLLOWER_CAN="${OPENARM_FOLLOWER_CAN:-${DEFAULT_FOLLOWER_CAN}}"
RECORD_PATH="${OPENARM_RECORD_PATH:-${PROJECT_DIR}/recordings/real_teleop_record.csv}"
POSITION_DUMP_PATH="${OPENARM_POSITION_DUMP_PATH:-${PROJECT_DIR}/recordings/motor_position_snapshots.txt}"
RECORD_START_PORT="${OPENARM_RECORD_START_PORT:-47920}"
LOG_PATH="${OPENARM_REAL_TELEOP_LOG:-/tmp/openarm_real_teleop.log}"
LIVE_RAMP_DURATION="${OPENARM_TELEOP_LIVE_RAMP_DURATION:-2.0}"
LIVE_VELOCITY_SCALE="${OPENARM_TELEOP_LIVE_VELOCITY_SCALE:-0.0}"
TELEOP_AUTO_HOME="${OPENARM_TELEOP_AUTO_HOME:-1}"
DEFAULT_TELEOP_HOME_POSE=""
if [ -f "${HOME}/Downloads/motor_p11.txt" ]; then
  DEFAULT_TELEOP_HOME_POSE="${HOME}/Downloads/motor_p11.txt"
elif [ -f "${WORKSPACE_DIR}/motor_p11.txt" ]; then
  DEFAULT_TELEOP_HOME_POSE="${WORKSPACE_DIR}/motor_p11.txt"
fi
TELEOP_HOME_POSE="${OPENARM_TELEOP_HOME_POSE:-${DEFAULT_TELEOP_HOME_POSE}}"
LIVE_WAIT_FOR_START="${OPENARM_TELEOP_LIVE_WAIT_FOR_START:-0}"
if [ -n "${OPENARM_TELEOP_MONITOR_PORT:-}" ]; then
  MONITOR_PORT="${OPENARM_TELEOP_MONITOR_PORT}"
elif [ "${ARM_SIDE}" = "left_arm" ]; then
  MONITOR_PORT=47951
else
  MONITOR_PORT=47950
fi

mkdir -p "$(dirname "${RECORD_PATH}")"
mkdir -p "$(dirname "${POSITION_DUMP_PATH}")"

request_sudo_password() {
  if sudo -n true >/dev/null 2>&1; then
    return
  fi
  if [ -n "${OPENARM_SUDO_PASSWORD:-}" ]; then
    return
  fi
  if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "[openarm][ERROR] sudo password is required but no terminal/display is available." >&2
    exit 1
  fi
  OPENARM_SUDO_PASSWORD="$(python3 - <<'PY'
import tkinter as tk
from tkinter import simpledialog

root = tk.Tk()
root.withdraw()
root.attributes("-topmost", True)
password = simpledialog.askstring(
    "OpenArm sudo password",
    "CAN 接口配置需要 sudo，请输入当前用户密码：",
    show="*",
    parent=root,
)
root.destroy()
print(password or "")
PY
)"
  export OPENARM_SUDO_PASSWORD
  if [ -z "${OPENARM_SUDO_PASSWORD}" ]; then
    echo "[openarm][ERROR] sudo password was not provided." >&2
    exit 1
  fi
}

run_sudo() {
  if sudo -n true >/dev/null 2>&1; then
    sudo "$@"
  else
    request_sudo_password
    printf '%s\n' "${OPENARM_SUDO_PASSWORD}" | sudo -S -p "" "$@"
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
  local configured=" "

  for iface in "$@"; do
    if [[ "${configured}" == *" ${iface} "* ]]; then
      continue
    fi
    configured="${configured}${iface} "

    if ! ip link show "${iface}" >/dev/null 2>&1; then
      echo "[openarm][ERROR] SocketCAN interface '${iface}' was not found or is not accessible." >&2
      exit 1
    fi

    echo "[openarm] configuring ${iface} as CAN-FD (${bitrate}/${dbitrate})"
    run_sudo ip link set "${iface}" down
    run_sudo ip link set "${iface}" type can bitrate "${bitrate}" dbitrate "${dbitrate}" fd on
    run_sudo ip link set "${iface}" up
  done
}

check_can_interface() {
  local iface="$1"
  if command -v ip >/dev/null 2>&1; then
    if ! ip link show "${iface}" >/dev/null 2>&1; then
      echo "[openarm][ERROR] SocketCAN interface '${iface}' was not found or is not accessible." >&2
      echo "[openarm][HINT] Check interface names with: ip link show type can" >&2
      exit 1
    fi
    if ! ip link show "${iface}" | grep -q "state UP"; then
      echo "[openarm][ERROR] SocketCAN interface '${iface}' exists but is not UP." >&2
      echo "[openarm][HINT] Re-run this script with OPENARM_AUTO_CONFIG_CAN=1." >&2
      exit 1
    fi
  fi
}

if [ ! -x "${PROJECT_DIR}/build/unilateral_control" ]; then
  cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build"
  cmake --build "${PROJECT_DIR}/build" -j2
fi

configure_can_interfaces "${LEADER_CAN}" "${FOLLOWER_CAN}"
check_can_interface "${LEADER_CAN}"
check_can_interface "${FOLLOWER_CAN}"

echo "[openarm] mode: real hardware C++ leader-follower teleop with CSV record"
echo "[openarm] URDF: ${URDF}"
echo "[openarm] arm_side=${ARM_SIDE} leader_can=${LEADER_CAN} follower_can=${FOLLOWER_CAN}"
echo "[openarm] record: ${RECORD_PATH}"
echo "[openarm] position snapshots: ${POSITION_DUMP_PATH}"
echo "[openarm] record_start_port: ${RECORD_START_PORT}"
echo "[openarm] live ramp duration: ${LIVE_RAMP_DURATION}s"
echo "[openarm] live velocity scale: ${LIVE_VELOCITY_SCALE}"
echo "[openarm] startup auto home: ${TELEOP_AUTO_HOME}"
echo "[openarm] startup home pose: ${TELEOP_HOME_POSE:-INITIAL_POSITION}"
echo "[openarm] live wait for start: ${LIVE_WAIT_FOR_START}"
echo "[openarm] monitor port: ${MONITOR_PORT}"

EXTRA_CONTROL_ARGS=()
if [ "${TELEOP_AUTO_HOME}" != "1" ]; then
  EXTRA_CONTROL_ARGS+=(--no-auto-home)
elif [ -n "${TELEOP_HOME_POSE}" ]; then
  EXTRA_CONTROL_ARGS+=(--home-pose "${TELEOP_HOME_POSE}")
fi
if [ "${MONITOR_PORT}" != "0" ]; then
  EXTRA_CONTROL_ARGS+=(--monitor-port "${MONITOR_PORT}")
fi

"${PROJECT_DIR}/build/unilateral_control" "${URDF}" "${URDF}" "${ARM_SIDE}" \
  "${LEADER_CAN}" "${FOLLOWER_CAN}" \
  --reference-source live --record "${RECORD_PATH}" \
  --record-start-port "${RECORD_START_PORT}" \
  --live-ramp-duration "${LIVE_RAMP_DURATION}" \
  --live-velocity-scale "${LIVE_VELOCITY_SCALE}" \
  --live-wait-for-start "${LIVE_WAIT_FOR_START}" \
  "${EXTRA_CONTROL_ARGS[@]}" \
  --position-dump "${POSITION_DUMP_PATH}" "$@" >"${LOG_PATH}" 2>&1 &
TELEOP_PID=$!

cleanup() {
  if kill -0 "${TELEOP_PID}" >/dev/null 2>&1; then
    kill "${TELEOP_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

sleep 1
if ! kill -0 "${TELEOP_PID}" >/dev/null 2>&1; then
  echo "[openarm][ERROR] C++ real teleop exited early. Log: ${LOG_PATH}" >&2
  tail -n 80 "${LOG_PATH}" >&2 || true
  exit 1
fi
echo "[openarm] C++ real teleop log: ${LOG_PATH}"
if [ "${OPENARM_NO_RECORD_PANEL:-0}" = "1" ]; then
  echo "[openarm] record panel disabled; waiting for external start trigger on ${RECORD_START_PORT}"
  wait "${TELEOP_PID}"
else
  python "${SCRIPT_DIR}/record_start_panel.py" \
    --port "${RECORD_START_PORT}" \
    --log "${LOG_PATH}" \
    --position-output "${POSITION_DUMP_PATH}" \
    --monitor-port "${MONITOR_PORT}" \
    --arm-side "${ARM_SIDE}"
fi
