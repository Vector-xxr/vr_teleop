#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${PROJECT_DIR}/.." && pwd)"
export LD_LIBRARY_PATH="/home/wenjiajie/anaconda3/lib:/home/wenjiajie/anaconda3/envs/openarm_teleop1/lib:${LD_LIBRARY_PATH:-}"

CSV="${1:-${OPENARM_REPLAY_PATH:-}}"
if [ -z "${CSV}" ]; then
  echo "Usage: $0 RECORDING.csv_or_dir [extra unilateral_control args...]" >&2
  echo "Or set OPENARM_REPLAY_PATH=recordings/file.csv" >&2
  exit 2
fi
if [ "$#" -gt 0 ]; then
  shift
fi

if [[ "${CSV}" != /* ]]; then
  CSV="${PROJECT_DIR}/${CSV}"
fi

URDF="${OPENARM_TELEOP_URDF:-${WORKSPACE_DIR}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
ARM_SIDE="${OPENARM_TELEOP_ARM_SIDE:-right_arm}"
if [ "${ARM_SIDE}" = "left_arm" ]; then
  DEFAULT_LEADER_CAN="can3"
  DEFAULT_FOLLOWER_CAN="can0"
else
  DEFAULT_LEADER_CAN="can2"
  DEFAULT_FOLLOWER_CAN="can1"
fi
LEADER_CAN="${OPENARM_LEADER_CAN:-${DEFAULT_LEADER_CAN}}"
FOLLOWER_CAN="${OPENARM_FOLLOWER_CAN:-${DEFAULT_FOLLOWER_CAN}}"
FOLLOWER_ONLY="${OPENARM_FOLLOWER_ONLY:-0}"

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

if [ ! -f "${CSV}" ] && [ ! -d "${CSV}" ]; then
  echo "[openarm][ERROR] replay path not found: ${CSV}" >&2
  exit 1
fi

if [ -f "${CSV}" ] && [ ! -s "${CSV}" ]; then
  echo "[openarm][ERROR] replay CSV is empty: ${CSV}" >&2
  exit 1
fi

if [ ! -x "${PROJECT_DIR}/build/unilateral_control" ]; then
  cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build"
  cmake --build "${PROJECT_DIR}/build" -j2
fi

if [ "${FOLLOWER_ONLY}" = "1" ]; then
  configure_can_interfaces "${FOLLOWER_CAN}"
  check_can_interface "${FOLLOWER_CAN}"
else
  configure_can_interfaces "${LEADER_CAN}" "${FOLLOWER_CAN}"
  check_can_interface "${LEADER_CAN}"
  check_can_interface "${FOLLOWER_CAN}"
fi

echo "[openarm] mode: real hardware C++ follower replay"
echo "[openarm] URDF: ${URDF}"
echo "[openarm] arm_side=${ARM_SIDE} leader_can=${LEADER_CAN} follower_can=${FOLLOWER_CAN}"
echo "[openarm] follower_only=${FOLLOWER_ONLY}"
echo "[openarm] replay: ${CSV}"
echo "[openarm][WARN] Test replay first with a small, slow CSV and clear workspace."

EXTRA_ARGS=()
if [ "${FOLLOWER_ONLY}" = "1" ]; then
  EXTRA_ARGS+=(--follower-only)
fi

exec "${PROJECT_DIR}/build/unilateral_control" "${URDF}" "${URDF}" "${ARM_SIDE}" \
  "${LEADER_CAN}" "${FOLLOWER_CAN}" \
  --reference-source replay --replay "${CSV}" "${EXTRA_ARGS[@]}" "$@"
