#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${PROJECT_DIR}/.." && pwd)"
RUN_SINGLE="${SCRIPT_DIR}/run_cpp_real_replay.sh"

SESSIONS_DIR="${OPENARM_RECORDINGS_SESSIONS_DIR:-${PROJECT_DIR}/recordings/sessions}"
DUAL_REPLAY_DIR=""
DUAL_MERGED_REPLAY_PATH=""
RIGHT_REPLAY_PATH=""
LEFT_REPLAY_PATH=""
EXTRA_ARGS=()

if [ "$#" -eq 0 ]; then
  if [ -d "${SESSIONS_DIR}" ]; then
    DUAL_REPLAY_DIR="$(find "${SESSIONS_DIR}" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-)"
    if [ -z "${DUAL_REPLAY_DIR}" ]; then
      echo "[openarm][ERROR] no session directories found in ${SESSIONS_DIR}" >&2
      exit 1
    fi
  else
    RIGHT_REPLAY_PATH="${OPENARM_RIGHT_REPLAY_PATH:-${PROJECT_DIR}/recordings/real_teleop_split/right/real_teleop_right.csv}"
    LEFT_REPLAY_PATH="${OPENARM_LEFT_REPLAY_PATH:-${PROJECT_DIR}/recordings/real_teleop_split/left/real_teleop_left.csv}"
  fi
elif [ "$#" -ge 1 ] && [ -d "$1" ]; then
  DUAL_REPLAY_DIR="$1"
  shift 1
  EXTRA_ARGS=("$@")
elif [ "$#" -ge 2 ] && [ -f "$1" ] && [ -f "$2" ]; then
  RIGHT_REPLAY_PATH="$1"
  LEFT_REPLAY_PATH="$2"
  shift 2
  EXTRA_ARGS=("$@")
elif [ "$#" -ge 1 ] && [ -f "$1" ]; then
  DUAL_MERGED_REPLAY_PATH="$1"
  shift 1
  EXTRA_ARGS=("$@")
elif [ "$#" -ge 2 ]; then
  RIGHT_REPLAY_PATH="$1"
  LEFT_REPLAY_PATH="$2"
  shift 2
  EXTRA_ARGS=("$@")
else
  echo "Usage: $0 [session_dir|sessions_root|RIGHT.csv LEFT.csv [extra args...]]" >&2
  exit 2
fi

DUAL_TRANSITION_DURATION="${OPENARM_DUAL_REPLAY_TRANSITION_DURATION:-5.0}"
DUAL_INITIAL_TRANSITION_DURATION="${OPENARM_DUAL_REPLAY_INITIAL_TRANSITION_DURATION:-3.0}"
DEFAULT_REPLAY_ARGS=(--initial-transition-duration "${DUAL_INITIAL_TRANSITION_DURATION}" --transition-duration "${DUAL_TRANSITION_DURATION}")
SYNC_START_DELAY="${OPENARM_REPLAY_SYNC_START_DELAY:-1.5}"
DUAL_REPLAY_SPEED="${OPENARM_DUAL_REPLAY_SPEED:-1.0}"
REPLAY_INIT_HOLD_DURATION="${OPENARM_REPLAY_INIT_HOLD_DURATION:-0.5}"
REPLAY_STARTUP_GAIN_RAMP_DURATION="${OPENARM_REPLAY_STARTUP_GAIN_RAMP_DURATION:-1.0}"
REPLAY_STARTUP_GAIN_RAMP_START_SCALE="${OPENARM_REPLAY_STARTUP_GAIN_RAMP_START_SCALE:-0.1}"
TORQUE_RECOVERY_SPEED="${OPENARM_TORQUE_RECOVERY_SPEED:-0.25}"
TORQUE_RECOVERY_MIN_DURATION="${OPENARM_TORQUE_RECOVERY_MIN_DURATION:-3.0}"
TORQUE_PANEL_ENABLED="${OPENARM_TORQUE_PANEL:-1}"
TORQUE_TELEMETRY_PORT="${OPENARM_TORQUE_TELEMETRY_PORT:-47940}"
TORQUE_TELEMETRY_OUTPUT="${OPENARM_TORQUE_TELEMETRY_OUTPUT:-}"
REPLAY_AUTO_HOME="${OPENARM_REPLAY_AUTO_HOME:-1}"
DEFAULT_REPLAY_POSE="${WORKSPACE_DIR}/motor_p11.txt"
REPLAY_STARTUP_POSE="${OPENARM_REPLAY_STARTUP_POSE:-${DEFAULT_REPLAY_POSE}}"
REPLAY_HOME_POSE="${OPENARM_REPLAY_HOME_POSE:-${DEFAULT_REPLAY_POSE}}"

RIGHT_LOG="${OPENARM_RIGHT_REAL_REPLAY_LOG:-/tmp/openarm_real_replay_right.log}"
LEFT_LOG="${OPENARM_LEFT_REAL_REPLAY_LOG:-/tmp/openarm_real_replay_left.log}"
RIGHT_REPLAY_START_PORT="${OPENARM_RIGHT_REPLAY_START_PORT:-47930}"
LEFT_REPLAY_START_PORT="${OPENARM_LEFT_REPLAY_START_PORT:-47931}"
RIGHT_LEADER_CAN="${OPENARM_RIGHT_LEADER_CAN:-leader_right}"
RIGHT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-follower_right}"
LEFT_LEADER_CAN="${OPENARM_LEFT_LEADER_CAN:-leader_left}"
LEFT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-follower_left}"

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

resolve_path() {
  local path="$1"
  if [[ "${path}" != /* ]]; then
    path="${PROJECT_DIR}/${path}"
  fi
  printf '%s\n' "${path}"
}

find_merged_replay() {
  local root="$1"
  local candidate=""

  for name in real_teleop_dual_merged.csv dual_merged.csv merged.csv; do
    if [ -f "${root}/${name}" ]; then
      printf '%s\n' "${root}/${name}"
      return 0
    fi
  done

  for child in "${root}"/*; do
    if [ ! -d "${child}" ]; then
      continue
    fi
    for name in real_teleop_dual_merged.csv dual_merged.csv merged.csv; do
      if [ -f "${child}/${name}" ]; then
        candidate="${child}/${name}"
      fi
    done
  done

  if [ -n "${candidate}" ]; then
    printf '%s\n' "${candidate}"
    return 0
  fi
  return 1
}

if [ -n "${DUAL_REPLAY_DIR}" ]; then
  DUAL_REPLAY_DIR="$(resolve_path "${DUAL_REPLAY_DIR}")"
  if [ -z "${TORQUE_TELEMETRY_OUTPUT}" ]; then
    TORQUE_TELEMETRY_OUTPUT="${DUAL_REPLAY_DIR}/torque_telemetry.csv"
  fi
  PREPARE_LOG="$(mktemp /tmp/openarm_dual_replay_prepare.XXXXXX.log)"
  python "${SCRIPT_DIR}/prepare_dual_replay_directory.py" "${DUAL_REPLAY_DIR}" | tee "${PREPARE_LOG}"
  if [ "${OPENARM_DUAL_REPLAY_SINGLE_PROCESS:-1}" = "1" ]; then
    DUAL_MERGED_REPLAY_PATH="$(grep '^MERGED_DIR=' "${PREPARE_LOG}" | tail -n 1 | cut -d= -f2-)"
  fi
  if [ -z "${DUAL_MERGED_REPLAY_PATH}" ]; then
    RIGHT_REPLAY_PATH="$(grep '^RIGHT_DIR=' "${PREPARE_LOG}" | tail -n 1 | cut -d= -f2-)"
    LEFT_REPLAY_PATH="$(grep '^LEFT_DIR=' "${PREPARE_LOG}" | tail -n 1 | cut -d= -f2-)"
  fi
  rm -f "${PREPARE_LOG}"
elif [ -n "${DUAL_MERGED_REPLAY_PATH}" ]; then
  DUAL_MERGED_REPLAY_PATH="$(resolve_path "${DUAL_MERGED_REPLAY_PATH}")"
  if [ -z "${TORQUE_TELEMETRY_OUTPUT}" ]; then
    TORQUE_TELEMETRY_OUTPUT="$(dirname "${DUAL_MERGED_REPLAY_PATH}")/torque_telemetry.csv"
  fi
else
  RIGHT_REPLAY_PATH="$(resolve_path "${RIGHT_REPLAY_PATH}")"
  LEFT_REPLAY_PATH="$(resolve_path "${LEFT_REPLAY_PATH}")"
fi

check_replay_path() {
  local label="$1"
  local path="$2"
  if [ ! -f "${path}" ] && [ ! -d "${path}" ]; then
    echo "[openarm][ERROR] ${label} replay path not found: ${path}" >&2
    exit 1
  fi
  if [ -f "${path}" ] && [ ! -s "${path}" ]; then
    echo "[openarm][ERROR] ${label} replay CSV is empty: ${path}" >&2
    exit 1
  fi
}

if [ -n "${DUAL_MERGED_REPLAY_PATH}" ]; then
  check_replay_path "merged" "${DUAL_MERGED_REPLAY_PATH}"
else
  check_replay_path "right" "${RIGHT_REPLAY_PATH}"
  check_replay_path "left" "${LEFT_REPLAY_PATH}"
fi

if [ -n "${DUAL_MERGED_REPLAY_PATH}" ]; then
  configure_can_interfaces "${RIGHT_FOLLOWER_CAN}" "${LEFT_FOLLOWER_CAN}"

  DUAL_BIN="${PROJECT_DIR}/build/dual_replay_control"
  URDF="${OPENARM_TELEOP_URDF:-${WORKSPACE_DIR}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
  if [ ! -x "${DUAL_BIN}" ]; then
    cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build"
    cmake --build "${PROJECT_DIR}/build" -j2
  fi

  echo "[openarm] mode: single-process dual real replay"
  echo "[openarm] merged replay: ${DUAL_MERGED_REPLAY_PATH}"
  echo "[openarm] left follower can : ${LEFT_FOLLOWER_CAN}"
  echo "[openarm] right follower can: ${RIGHT_FOLLOWER_CAN}"
  echo "[openarm] initial transition duration: ${DUAL_INITIAL_TRANSITION_DURATION}s (0.0 = adaptive shortest)"
  echo "[openarm] synchronized start delay: ${SYNC_START_DELAY}s"
  echo "[openarm] replay speed: ${DUAL_REPLAY_SPEED}x"
  echo "[openarm] init hold duration: ${REPLAY_INIT_HOLD_DURATION}s"
  echo "[openarm] startup gain ramp: ${REPLAY_STARTUP_GAIN_RAMP_START_SCALE} -> 1.0 over ${REPLAY_STARTUP_GAIN_RAMP_DURATION}s"
  echo "[openarm] torque safety recovery: ${TORQUE_RECOVERY_SPEED} rad/s, min ${TORQUE_RECOVERY_MIN_DURATION}s"
  echo "[openarm] startup auto home: ${REPLAY_AUTO_HOME}"
  echo "[openarm] startup pose: ${REPLAY_STARTUP_POSE:-INITIAL_POSITION}"
  echo "[openarm] startup home pose: ${REPLAY_HOME_POSE}"
  if [ "${TORQUE_PANEL_ENABLED}" = "1" ]; then
    echo "[openarm] opening torque telemetry panel on UDP port ${TORQUE_TELEMETRY_PORT}"
    echo "[openarm] torque telemetry CSV: ${TORQUE_TELEMETRY_OUTPUT}"
    python "${SCRIPT_DIR}/torque_telemetry_panel.py" \
      --port "${TORQUE_TELEMETRY_PORT}" \
      --output "${TORQUE_TELEMETRY_OUTPUT}" &
    TORQUE_PANEL_PID=$!
  else
    TORQUE_PANEL_PID=""
    TORQUE_TELEMETRY_PORT=0
  fi

  cleanup_single_process() {
    if [ -n "${TORQUE_PANEL_PID:-}" ] && kill -0 "${TORQUE_PANEL_PID}" >/dev/null 2>&1; then
      echo "[openarm] torque telemetry panel left open for inspection (pid=${TORQUE_PANEL_PID})"
    fi
  }
  trap cleanup_single_process EXIT INT TERM

  DUAL_CONTROL_ARGS=()
  if [ "${REPLAY_AUTO_HOME}" != "1" ]; then
    DUAL_CONTROL_ARGS+=(--no-auto-home)
  else
    if [ -n "${REPLAY_STARTUP_POSE}" ]; then
      DUAL_CONTROL_ARGS+=(--startup-pose "${REPLAY_STARTUP_POSE}")
    fi
    if [ -n "${REPLAY_HOME_POSE}" ]; then
      DUAL_CONTROL_ARGS+=(--home-pose "${REPLAY_HOME_POSE}")
    fi
  fi

  "${DUAL_BIN}" \
    --urdf "${URDF}" \
    --replay "${DUAL_MERGED_REPLAY_PATH}" \
    --left-follower-can "${LEFT_FOLLOWER_CAN}" \
    --right-follower-can "${RIGHT_FOLLOWER_CAN}" \
    --initial-transition-duration "${DUAL_INITIAL_TRANSITION_DURATION}" \
    --transition-duration "${DUAL_TRANSITION_DURATION}" \
    --transition 1 \
    --start-delay "${SYNC_START_DELAY}" \
    --speed "${DUAL_REPLAY_SPEED}" \
    --init-hold-duration "${REPLAY_INIT_HOLD_DURATION}" \
    --startup-gain-ramp-duration "${REPLAY_STARTUP_GAIN_RAMP_DURATION}" \
    --startup-gain-ramp-start-scale "${REPLAY_STARTUP_GAIN_RAMP_START_SCALE}" \
    --torque-recovery-speed "${TORQUE_RECOVERY_SPEED}" \
    --torque-recovery-min-duration "${TORQUE_RECOVERY_MIN_DURATION}" \
    --torque-telemetry-port "${TORQUE_TELEMETRY_PORT}" \
    "${DUAL_CONTROL_ARGS[@]}" \
    "${EXTRA_ARGS[@]}"
  exit $?
fi

RIGHT_PID=""
LEFT_PID=""

configure_can_interfaces "${RIGHT_LEADER_CAN}" "${RIGHT_FOLLOWER_CAN}" "${LEFT_LEADER_CAN}" "${LEFT_FOLLOWER_CAN}"

cleanup() {
  if [ -n "${RIGHT_PID}" ] && kill -0 "${RIGHT_PID}" >/dev/null 2>&1; then
    kill "${RIGHT_PID}" >/dev/null 2>&1 || true
  fi
  if [ -n "${LEFT_PID}" ] && kill -0 "${LEFT_PID}" >/dev/null 2>&1; then
    kill "${LEFT_PID}" >/dev/null 2>&1 || true
  fi
  wait "${RIGHT_PID:-0}" >/dev/null 2>&1 || true
  wait "${LEFT_PID:-0}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

wait_for_replay_ready() {
  local label="$1"
  local log_path="$2"
  local pid="$3"
  local timeout_s="${OPENARM_REPLAY_READY_TIMEOUT:-60}"
  local deadline=$((SECONDS + timeout_s))

  while [ "${SECONDS}" -lt "${deadline}" ]; do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      echo "[openarm][ERROR] ${label} replay exited before ready. Log: ${log_path}" >&2
      tail -n 80 "${log_path}" >&2 || true
      exit 1
    fi
    if [ -f "${log_path}" ] && grep -q "\[replay\] waiting for start trigger" "${log_path}"; then
      echo "[openarm] ${label} replay is ready for synchronized start"
      return
    fi
    sleep 0.1
  done

  echo "[openarm][ERROR] timeout waiting for ${label} replay to become ready. Log: ${log_path}" >&2
  tail -n 80 "${log_path}" >&2 || true
  exit 1
}

send_synchronized_start() {
  python3 - "${RIGHT_REPLAY_START_PORT}" "${LEFT_REPLAY_START_PORT}" <<'PY'
import socket
import sys

ports = [int(port) for port in sys.argv[1:]]
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    for port in ports:
        sock.sendto(b"start", ("127.0.0.1", port))
finally:
    sock.close()
PY
}

echo "[openarm] starting right replay: ${RIGHT_REPLAY_PATH}"
echo "[openarm] right can: leader=${RIGHT_LEADER_CAN} follower=${RIGHT_FOLLOWER_CAN} start_port=${RIGHT_REPLAY_START_PORT}"
SPLIT_CONTROL_ARGS=()
if [ "${REPLAY_AUTO_HOME}" != "1" ]; then
  SPLIT_CONTROL_ARGS+=(--no-auto-home)
fi
OPENARM_TELEOP_ARM_SIDE=right_arm \
OPENARM_LEADER_CAN="${RIGHT_LEADER_CAN}" \
OPENARM_FOLLOWER_CAN="${RIGHT_FOLLOWER_CAN}" \
OPENARM_SKIP_CAN_CONFIG=1 \
"${RUN_SINGLE}" "${RIGHT_REPLAY_PATH}" \
  --replay-start-port "${RIGHT_REPLAY_START_PORT}" \
  "${DEFAULT_REPLAY_ARGS[@]}" "${SPLIT_CONTROL_ARGS[@]}" "${EXTRA_ARGS[@]}" >"${RIGHT_LOG}" 2>&1 &
RIGHT_PID=$!

echo "[openarm] starting left replay: ${LEFT_REPLAY_PATH}"
echo "[openarm] left can: leader=${LEFT_LEADER_CAN} follower=${LEFT_FOLLOWER_CAN} start_port=${LEFT_REPLAY_START_PORT}"
OPENARM_TELEOP_ARM_SIDE=left_arm \
OPENARM_LEADER_CAN="${LEFT_LEADER_CAN}" \
OPENARM_FOLLOWER_CAN="${LEFT_FOLLOWER_CAN}" \
OPENARM_SKIP_CAN_CONFIG=1 \
"${RUN_SINGLE}" "${LEFT_REPLAY_PATH}" \
  --replay-start-port "${LEFT_REPLAY_START_PORT}" \
  "${DEFAULT_REPLAY_ARGS[@]}" "${SPLIT_CONTROL_ARGS[@]}" "${EXTRA_ARGS[@]}" >"${LEFT_LOG}" 2>&1 &
LEFT_PID=$!

echo "[openarm] waiting for both replays to finish initialization"
wait_for_replay_ready "right" "${RIGHT_LOG}" "${RIGHT_PID}"
wait_for_replay_ready "left" "${LEFT_LOG}" "${LEFT_PID}"
echo "[openarm] both replays ready; waiting ${SYNC_START_DELAY}s before synchronized start"
sleep "${SYNC_START_DELAY}"
echo "[openarm] sending synchronized replay start"
send_synchronized_start

echo "[openarm] both real replays are running from synchronized trigger"
echo "[openarm] initial transition duration: ${DUAL_INITIAL_TRANSITION_DURATION}s (0.0 = adaptive shortest)"
echo "[openarm] between-action transition duration: ${DUAL_TRANSITION_DURATION}s"
echo "[openarm] synchronized start delay: ${SYNC_START_DELAY}s"
echo "[openarm] startup auto home: ${REPLAY_AUTO_HOME}"
echo "[openarm] right log: ${RIGHT_LOG}"
echo "[openarm] left log : ${LEFT_LOG}"
echo "[openarm] press Ctrl+C to stop both replays"

while true; do
  if ! kill -0 "${RIGHT_PID}" >/dev/null 2>&1; then
    echo "[openarm] right replay exited; stopping left"
    exit 0
  fi
  if ! kill -0 "${LEFT_PID}" >/dev/null 2>&1; then
    echo "[openarm] left replay exited; stopping right"
    exit 0
  fi
  sleep 1
done
