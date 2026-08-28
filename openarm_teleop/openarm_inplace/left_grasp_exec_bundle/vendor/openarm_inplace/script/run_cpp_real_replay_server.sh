#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${PROJECT_DIR}/.." && pwd)"
CONDA_LIB_DIR="${CONDA_PREFIX:-/home/knowin/miniconda3/envs/openarm_teleop1}/lib"
CONDA_ROOT_LIB_DIR="$(cd "${CONDA_LIB_DIR}/../.." && pwd)/lib"
export LD_LIBRARY_PATH="${CONDA_ROOT_LIB_DIR}:${CONDA_LIB_DIR}:${LD_LIBRARY_PATH:-}"

URDF="${OPENARM_TELEOP_URDF:-${WORKSPACE_DIR}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
LEFT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-can1}"
RIGHT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-can0}"
COMMAND_PORT="${OPENARM_REPLAY_COMMAND_PORT:-47970}"
STATUS_PORT="${OPENARM_REPLAY_STATUS_PORT:-47971}"
REPLAY_AUTO_HOME="${OPENARM_REPLAY_AUTO_HOME:-1}"
DEFAULT_REPLAY_POSE="${WORKSPACE_DIR}/motor_p11.txt"
REPLAY_STARTUP_POSE="${OPENARM_REPLAY_STARTUP_POSE:-${DEFAULT_REPLAY_POSE}}"
REPLAY_HOME_POSE="${OPENARM_REPLAY_HOME_POSE:-${DEFAULT_REPLAY_POSE}}"
DUAL_INITIAL_TRANSITION_DURATION="${OPENARM_DUAL_REPLAY_INITIAL_TRANSITION_DURATION:-3.0}"
DUAL_TRANSITION_DURATION="${OPENARM_DUAL_REPLAY_TRANSITION_DURATION:-5.0}"
DUAL_REPLAY_SPEED="${OPENARM_DUAL_REPLAY_SPEED:-1.0}"
REPLAY_INIT_HOLD_DURATION="${OPENARM_REPLAY_INIT_HOLD_DURATION:-0.5}"
REPLAY_STARTUP_GAIN_RAMP_DURATION="${OPENARM_REPLAY_STARTUP_GAIN_RAMP_DURATION:-1.0}"
REPLAY_STARTUP_GAIN_RAMP_START_SCALE="${OPENARM_REPLAY_STARTUP_GAIN_RAMP_START_SCALE:-0.1}"
TORQUE_RECOVERY_SPEED="${OPENARM_TORQUE_RECOVERY_SPEED:-0.25}"
TORQUE_RECOVERY_MIN_DURATION="${OPENARM_TORQUE_RECOVERY_MIN_DURATION:-3.0}"

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
      echo "[openarm][ERROR] SocketCAN interface '${iface}' was not found or is not accessible." >&2
      exit 1
    fi
    echo "[openarm] configuring ${iface} as CAN-FD (${bitrate}/${dbitrate})"
    run_sudo ip link set "${iface}" down
    run_sudo ip link set "${iface}" type can bitrate "${bitrate}" dbitrate "${dbitrate}" fd on
    run_sudo ip link set "${iface}" up
  done
}

run_sudo() {
  if sudo -n true >/dev/null 2>&1; then
    sudo "$@"
  else
    request_sudo_password
    printf '%s\n' "${OPENARM_SUDO_PASSWORD}" | sudo -S -p "" "$@"
  fi
}

DUAL_BUILD_DIR="${OPENARM_DUAL_BUILD_DIR:-${PROJECT_DIR}/build_current}"
DUAL_BIN="${DUAL_BUILD_DIR}/dual_replay_control"
NEED_BUILD=0
if [ ! -x "${DUAL_BIN}" ]; then
  NEED_BUILD=1
else
  for src in \
    "${PROJECT_DIR}/src/openarm_constants.hpp" \
    "${PROJECT_DIR}/control/openarm_dual_replay_control.cpp" \
    "${PROJECT_DIR}/src/common.hpp"
  do
    if [ -f "${src}" ] && [ "${src}" -nt "${DUAL_BIN}" ]; then
      NEED_BUILD=1
      echo "[openarm] ${src} newer than ${DUAL_BIN}; rebuilding dual_replay_control"
      break
    fi
  done
fi
if [ "${NEED_BUILD}" = "1" ]; then
  cmake -S "${PROJECT_DIR}" -B "${DUAL_BUILD_DIR}"
  cmake --build "${DUAL_BUILD_DIR}" --target dual_replay_control -j2
fi

configure_can_interfaces "${RIGHT_FOLLOWER_CAN}" "${LEFT_FOLLOWER_CAN}"

CONTROL_ARGS=()
if [ "${REPLAY_AUTO_HOME}" != "1" ]; then
  CONTROL_ARGS+=(--no-auto-home)
else
  if [ -n "${REPLAY_STARTUP_POSE}" ]; then
    CONTROL_ARGS+=(--startup-pose "${REPLAY_STARTUP_POSE}")
  fi
  if [ -n "${REPLAY_HOME_POSE}" ]; then
    CONTROL_ARGS+=(--home-pose "${REPLAY_HOME_POSE}")
  fi
fi

echo "[openarm] mode: persistent dual replay server"
echo "[openarm] command port: ${COMMAND_PORT}"
echo "[openarm] status port: ${STATUS_PORT}"
echo "[openarm] initial transition: ${DUAL_INITIAL_TRANSITION_DURATION}s"
echo "[openarm] init hold duration: ${REPLAY_INIT_HOLD_DURATION}s"
echo "[openarm] startup gain ramp: ${REPLAY_STARTUP_GAIN_RAMP_START_SCALE} -> 1.0 over ${REPLAY_STARTUP_GAIN_RAMP_DURATION}s"
echo "[openarm] startup pose: ${REPLAY_STARTUP_POSE:-INITIAL_POSITION}"
echo "[openarm] startup home pose: ${REPLAY_HOME_POSE}"

unset OPENARM_SUDO_PASSWORD

exec "${DUAL_BIN}" \
  --urdf "${URDF}" \
  --left-follower-can "${LEFT_FOLLOWER_CAN}" \
  --right-follower-can "${RIGHT_FOLLOWER_CAN}" \
  --command-port "${COMMAND_PORT}" \
  --status-port "${STATUS_PORT}" \
  --initial-transition-duration "${DUAL_INITIAL_TRANSITION_DURATION}" \
  --transition-duration "${DUAL_TRANSITION_DURATION}" \
  --transition 1 \
  --start-delay 0 \
  --speed "${DUAL_REPLAY_SPEED}" \
  --init-hold-duration "${REPLAY_INIT_HOLD_DURATION}" \
  --startup-gain-ramp-duration "${REPLAY_STARTUP_GAIN_RAMP_DURATION}" \
  --startup-gain-ramp-start-scale "${REPLAY_STARTUP_GAIN_RAMP_START_SCALE}" \
  --torque-recovery-speed "${TORQUE_RECOVERY_SPEED}" \
  --torque-recovery-min-duration "${TORQUE_RECOVERY_MIN_DURATION}" \
  "${CONTROL_ARGS[@]}"
