#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${PROJECT_DIR}/.." && pwd)"
RUN_SINGLE="${SCRIPT_DIR}/run_cpp_real_teleop_record.sh"

RECORD_NAME="${1:-${OPENARM_RECORD_NAME:-}}"
if [ -z "${RECORD_NAME}" ]; then
  RECORD_NAME="$(date +real_%Y%m%d_%H%M%S)"
fi
RECORD_NAME="${RECORD_NAME// /_}"
SESSION_DIR="${OPENARM_RECORD_SESSION_DIR:-${PROJECT_DIR}/recordings/sessions/${RECORD_NAME}}"

RIGHT_RECORD_PATH="${OPENARM_RIGHT_RECORD_PATH:-${SESSION_DIR}/real_teleop_split/right/real_teleop_right.csv}"
LEFT_RECORD_PATH="${OPENARM_LEFT_RECORD_PATH:-${SESSION_DIR}/real_teleop_split/left/real_teleop_left.csv}"
MERGED_RECORD_PATH="${OPENARM_MERGED_RECORD_PATH:-${SESSION_DIR}/real_teleop_dual_merged.csv}"
POSITION_DUMP_PATH="${OPENARM_POSITION_DUMP_PATH:-${SESSION_DIR}/motor_position_snapshots.txt}"
MERGED_RECORD_HZ="${OPENARM_MERGED_RECORD_HZ:-1000}"
RIGHT_RECORD_START_PORT="${OPENARM_RIGHT_RECORD_START_PORT:-47920}"
LEFT_RECORD_START_PORT="${OPENARM_LEFT_RECORD_START_PORT:-47921}"
RIGHT_LOG="${OPENARM_RIGHT_REAL_TELEOP_LOG:-/tmp/openarm_real_teleop_right.log}"
LEFT_LOG="${OPENARM_LEFT_REAL_TELEOP_LOG:-/tmp/openarm_real_teleop_left.log}"
TELEOP_MONITOR_ENABLED="${OPENARM_TELEOP_MONITOR_PANEL:-1}"
TELEOP_MONITOR_AUTO_OPEN="${OPENARM_TELEOP_MONITOR_AUTO_OPEN:-0}"
RIGHT_MONITOR_PORT="${OPENARM_RIGHT_TELEOP_MONITOR_PORT:-47950}"
LEFT_MONITOR_PORT="${OPENARM_LEFT_TELEOP_MONITOR_PORT:-47951}"

RIGHT_LEADER_CAN="${OPENARM_RIGHT_LEADER_CAN:-leader_right}"
RIGHT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-follower_right}"
LEFT_LEADER_CAN="${OPENARM_LEFT_LEADER_CAN:-leader_left}"
LEFT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-follower_left}"
RIGHT_TELEOP_HOME_POSE="${OPENARM_RIGHT_TELEOP_HOME_POSE:-${OPENARM_TELEOP_HOME_POSE:-}}"
LEFT_TELEOP_HOME_POSE="${OPENARM_LEFT_TELEOP_HOME_POSE:-${OPENARM_TELEOP_HOME_POSE:-}}"

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
      echo "[openarm][HINT] Check interface names with: ip link show type can" >&2
      exit 1
    fi

    echo "[openarm] configuring ${iface} as CAN-FD (${bitrate}/${dbitrate})"
    run_sudo ip link set "${iface}" down
    run_sudo ip link set "${iface}" type can bitrate "${bitrate}" dbitrate "${dbitrate}" fd on
    run_sudo ip link set "${iface}" up
  done
}

RIGHT_PID=""
LEFT_PID=""
MERGE_DONE=0

merge_recordings() {
  if [ "${MERGE_DONE}" = "1" ]; then
    return
  fi
  MERGE_DONE=1
  local right_dir left_dir right_base left_base right_stem left_stem right_ext left_ext merged_dir merged_base merged_stem merged_ext
  right_dir="$(dirname "${RIGHT_RECORD_PATH}")"
  left_dir="$(dirname "${LEFT_RECORD_PATH}")"
  right_base="$(basename "${RIGHT_RECORD_PATH}")"
  left_base="$(basename "${LEFT_RECORD_PATH}")"
  right_ext=".${right_base##*.}"
  left_ext=".${left_base##*.}"
  right_stem="${right_base%${right_ext}}"
  left_stem="${left_base%${left_ext}}"
  merged_dir="$(dirname "${MERGED_RECORD_PATH}")"
  merged_base="$(basename "${MERGED_RECORD_PATH}")"
  merged_ext=".${merged_base##*.}"
  merged_stem="${merged_base%${merged_ext}}"

  local found_segments=0
  shopt -s nullglob
  for right_segment in "${right_dir}/${right_stem}"_[0-9][0-9]"${right_ext}"; do
    local segment suffix left_segment merged_segment
    segment="$(basename "${right_segment}")"
    suffix="${segment#${right_stem}_}"
    suffix="${suffix%${right_ext}}"
    left_segment="${left_dir}/${left_stem}_${suffix}${left_ext}"
    merged_segment="${merged_dir}/${merged_stem}_${suffix}${merged_ext}"
    if [ -s "${right_segment}" ] && [ -s "${left_segment}" ]; then
      found_segments=1
      python "${SCRIPT_DIR}/merge_dual_real_recordings.py" \
        --left "${left_segment}" \
        --right "${right_segment}" \
        --output "${merged_segment}" \
        --hz "${MERGED_RECORD_HZ}" || true
    else
      echo "[openarm][WARN] skip segment ${suffix}: left/right segment missing or empty"
    fi
  done
  shopt -u nullglob
  if [ "${found_segments}" = "1" ]; then
    return
  fi

  if [ -s "${LEFT_RECORD_PATH}" ] && [ -s "${RIGHT_RECORD_PATH}" ]; then
    python "${SCRIPT_DIR}/merge_dual_real_recordings.py"       --left "${LEFT_RECORD_PATH}"       --right "${RIGHT_RECORD_PATH}"       --output "${MERGED_RECORD_PATH}"       --hz "${MERGED_RECORD_HZ}" || true
  else
    echo "[openarm][WARN] skip merge: left/right recording file is missing or empty"
  fi
}

cleanup() {
  if [ -n "${RIGHT_PID}" ] && kill -0 "${RIGHT_PID}" >/dev/null 2>&1; then
    kill "${RIGHT_PID}" >/dev/null 2>&1 || true
  fi
  if [ -n "${LEFT_PID}" ] && kill -0 "${LEFT_PID}" >/dev/null 2>&1; then
    kill "${LEFT_PID}" >/dev/null 2>&1 || true
  fi
  wait "${RIGHT_PID:-0}" >/dev/null 2>&1 || true
  if [ -n "${PANEL_PID:-}" ] && kill -0 "${PANEL_PID}" >/dev/null 2>&1; then
    kill "${PANEL_PID}" >/dev/null 2>&1 || true
  fi
  if [ -n "${MONITOR_PANEL_PID:-}" ] && kill -0 "${MONITOR_PANEL_PID}" >/dev/null 2>&1; then
    kill "${MONITOR_PANEL_PID}" >/dev/null 2>&1 || true
  fi
  wait "${LEFT_PID:-0}" >/dev/null 2>&1 || true
  wait "${PANEL_PID:-0}" >/dev/null 2>&1 || true
  wait "${MONITOR_PANEL_PID:-0}" >/dev/null 2>&1 || true
  merge_recordings
}
trap cleanup EXIT INT TERM

configure_can_interfaces "${RIGHT_LEADER_CAN}" "${RIGHT_FOLLOWER_CAN}" "${LEFT_LEADER_CAN}" "${LEFT_FOLLOWER_CAN}"

if [ "${TELEOP_MONITOR_ENABLED}" != "1" ]; then
  MONITOR_PANEL_PID=""
  RIGHT_MONITOR_PORT=0
  LEFT_MONITOR_PORT=0
elif [ "${TELEOP_MONITOR_AUTO_OPEN}" = "1" ]; then
  echo "[openarm] opening teleop monitor panel on UDP ports right=${RIGHT_MONITOR_PORT}, left=${LEFT_MONITOR_PORT}"
  python "${SCRIPT_DIR}/teleop_monitor_panel.py" \
    --right-port "${RIGHT_MONITOR_PORT}" \
    --left-port "${LEFT_MONITOR_PORT}" &
  MONITOR_PANEL_PID=$!
else
  MONITOR_PANEL_PID=""
fi

echo "[openarm] starting right arm: leader=${RIGHT_LEADER_CAN} follower=${RIGHT_FOLLOWER_CAN} port=${RIGHT_RECORD_START_PORT}"
OPENARM_TELEOP_ARM_SIDE=right_arm \
OPENARM_LEADER_CAN="${RIGHT_LEADER_CAN}" \
OPENARM_FOLLOWER_CAN="${RIGHT_FOLLOWER_CAN}" \
OPENARM_TELEOP_HOME_POSE="${RIGHT_TELEOP_HOME_POSE}" \
OPENARM_RECORD_START_PORT="${RIGHT_RECORD_START_PORT}" \
OPENARM_REAL_TELEOP_LOG="${RIGHT_LOG}" \
OPENARM_RECORD_PATH="${RIGHT_RECORD_PATH}" \
OPENARM_TELEOP_MONITOR_PORT="${RIGHT_MONITOR_PORT}" \
OPENARM_SKIP_CAN_CONFIG=1 \
OPENARM_NO_RECORD_PANEL=1 \
"${RUN_SINGLE}" --position-dump "${POSITION_DUMP_PATH}" &
RIGHT_PID=$!

sleep 0.8

echo "[openarm] starting left arm: leader=${LEFT_LEADER_CAN} follower=${LEFT_FOLLOWER_CAN} port=${LEFT_RECORD_START_PORT}"
OPENARM_TELEOP_ARM_SIDE=left_arm \
OPENARM_LEADER_CAN="${LEFT_LEADER_CAN}" \
OPENARM_FOLLOWER_CAN="${LEFT_FOLLOWER_CAN}" \
OPENARM_TELEOP_HOME_POSE="${LEFT_TELEOP_HOME_POSE}" \
OPENARM_RECORD_START_PORT="${LEFT_RECORD_START_PORT}" \
OPENARM_REAL_TELEOP_LOG="${LEFT_LOG}" \
OPENARM_RECORD_PATH="${LEFT_RECORD_PATH}" \
OPENARM_TELEOP_MONITOR_PORT="${LEFT_MONITOR_PORT}" \
OPENARM_SKIP_CAN_CONFIG=1 \
OPENARM_NO_RECORD_PANEL=1 \
"${RUN_SINGLE}" --position-dump "${POSITION_DUMP_PATH}" &
LEFT_PID=$!

echo "[openarm] both teleop launchers are running"
echo "[openarm] record name: ${RECORD_NAME}"
echo "[openarm] session dir: ${SESSION_DIR}"
echo "[openarm] right record: ${RIGHT_RECORD_PATH}"
echo "[openarm] left record : ${LEFT_RECORD_PATH}"
echo "[openarm] merged record: ${MERGED_RECORD_PATH}"
echo "[openarm] position snapshots: ${POSITION_DUMP_PATH}"
echo "[openarm] right log: ${RIGHT_LOG}"
echo "[openarm] left log : ${LEFT_LOG}"
echo "[openarm] teleop monitor: ${TELEOP_MONITOR_ENABLED} auto_open=${TELEOP_MONITOR_AUTO_OPEN} right_port=${RIGHT_MONITOR_PORT} left_port=${LEFT_MONITOR_PORT}"
echo "[openarm] opening one panel to start both recordings"
python "${SCRIPT_DIR}/record_start_panel_both.py" \
  --right-port "${RIGHT_RECORD_START_PORT}" \
  --left-port "${LEFT_RECORD_START_PORT}" \
  --right-log "${RIGHT_LOG}" \
  --left-log "${LEFT_LOG}" \
  --position-output "${POSITION_DUMP_PATH}" \
  --monitor-right-port "${RIGHT_MONITOR_PORT}" \
  --monitor-left-port "${LEFT_MONITOR_PORT}" &
PANEL_PID=$!

echo "[openarm] close the dual record panel or press Ctrl+C here to stop both arms"

while true; do
  if ! kill -0 "${PANEL_PID}" >/dev/null 2>&1; then
    echo "[openarm] dual record panel closed; stopping both arms"
    exit 0
  fi
  if ! kill -0 "${RIGHT_PID}" >/dev/null 2>&1; then
    echo "[openarm][WARN] right arm launcher exited; stopping left"
    exit 1
  fi
  if ! kill -0 "${LEFT_PID}" >/dev/null 2>&1; then
    echo "[openarm][WARN] left arm launcher exited; stopping right"
    exit 1
  fi
  sleep 1
done
