#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RUN_SINGLE="${SCRIPT_DIR}/run_cpp_real_follower_record.sh"

# Match Quest human_mimic CAN + gripper defaults (record→replay).
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/openarm_vr_compat_env.sh"

RECORD_NAME="${1:-${OPENARM_RECORD_NAME:-}}"
if [ -z "${RECORD_NAME}" ]; then
  RECORD_NAME="$(date +follower_%Y%m%d_%H%M%S)"
fi
RECORD_NAME="${RECORD_NAME// /_}"
SESSION_DIR="${OPENARM_RECORD_SESSION_DIR:-${PROJECT_DIR}/recordings/sessions/${RECORD_NAME}}"

RIGHT_RECORD_PATH="${OPENARM_RIGHT_RECORD_PATH:-${SESSION_DIR}/real_follower_split/right/real_follower_right.csv}"
LEFT_RECORD_PATH="${OPENARM_LEFT_RECORD_PATH:-${SESSION_DIR}/real_follower_split/left/real_follower_left.csv}"
MERGED_RECORD_PATH="${OPENARM_MERGED_RECORD_PATH:-${SESSION_DIR}/real_follower_dual_merged.csv}"
POSITION_DUMP_PATH="${OPENARM_POSITION_DUMP_PATH:-${SESSION_DIR}/follower_position_snapshots.txt}"
MERGED_RECORD_HZ="${OPENARM_MERGED_RECORD_HZ:-1000}"

RIGHT_RECORD_START_PORT="${OPENARM_RIGHT_RECORD_START_PORT:-47920}"
LEFT_RECORD_START_PORT="${OPENARM_LEFT_RECORD_START_PORT:-47921}"
RIGHT_LOG="${OPENARM_RIGHT_REAL_FOLLOWER_RECORD_LOG:-/tmp/openarm_real_follower_record_right.log}"
LEFT_LOG="${OPENARM_LEFT_REAL_FOLLOWER_RECORD_LOG:-/tmp/openarm_real_follower_record_left.log}"
RIGHT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN}"
LEFT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN}"

mkdir -p "$(dirname "${POSITION_DUMP_PATH}")"

RIGHT_PID=""
LEFT_PID=""
PANEL_PID=""
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
    python "${SCRIPT_DIR}/merge_dual_real_recordings.py" \
      --left "${LEFT_RECORD_PATH}" \
      --right "${RIGHT_RECORD_PATH}" \
      --output "${MERGED_RECORD_PATH}" \
      --hz "${MERGED_RECORD_HZ}" || true
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
  if [ -n "${PANEL_PID}" ] && kill -0 "${PANEL_PID}" >/dev/null 2>&1; then
    kill "${PANEL_PID}" >/dev/null 2>&1 || true
  fi
  wait "${RIGHT_PID:-0}" >/dev/null 2>&1 || true
  wait "${LEFT_PID:-0}" >/dev/null 2>&1 || true
  wait "${PANEL_PID:-0}" >/dev/null 2>&1 || true
  merge_recordings
}
trap cleanup EXIT INT TERM

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
    echo "[openarm][ERROR] SocketCAN interface '${iface}' was not found or is not accessible." >&2
    echo "[openarm][HINT] Check interface names with: ip link show type can" >&2
    exit 1
  fi
  echo "[openarm] configuring ${iface} as CAN-FD (${bitrate}/${dbitrate})"
  run_sudo ip link set "${iface}" down
  run_sudo ip link set "${iface}" type can bitrate "${bitrate}" dbitrate "${dbitrate}" fd on
  run_sudo ip link set "${iface}" up
}

wait_recorder_ready() {
  local pid="$1"
  local log="$2"
  local label="$3"
  local i
  for i in $(seq 1 60); do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      echo "[openarm][ERROR] ${label} follower recorder exited before ready. Log: ${log}" >&2
      tail -n 80 "${log}" >&2 || true
      exit 1
    fi
    if grep -q "waiting for UDP start/stop/dump" "${log}" 2>/dev/null; then
      echo "[openarm] ${label} recorder ready (motors up, waiting for panel Start)"
      return 0
    fi
    sleep 0.25
  done
  echo "[openarm][ERROR] timed out waiting for ${label} recorder. Log: ${log}" >&2
  tail -n 80 "${log}" >&2 || true
  exit 1
}

# 1) Configure both CAN ports first (one sudo password prompt, then cache covers both).
# 2) Only then start recorders that connect/enable motors.
# 3) Open the panel after both report ready.
if [ "${OPENARM_SKIP_CAN_CONFIG:-0}" != "1" ] && [ "${OPENARM_AUTO_CONFIG_CAN:-1}" = "1" ]; then
  echo "[openarm] configuring CAN before connecting motors (sudo may ask for password)..."
  configure_one_can "${RIGHT_FOLLOWER_CAN}"
  configure_one_can "${LEFT_FOLLOWER_CAN}"
  echo "[openarm] CAN ready"
else
  echo "[openarm] skipping CAN auto-config (OPENARM_SKIP_CAN_CONFIG / OPENARM_AUTO_CONFIG_CAN)"
fi
unset OPENARM_SUDO_PASSWORD

FOLLOWER_BIN="${PROJECT_DIR}/build/follower_record_control"
if [ ! -x "${FOLLOWER_BIN}" ]; then
  cmake -S "${PROJECT_DIR}" -B "${PROJECT_DIR}/build"
  cmake --build "${PROJECT_DIR}/build" --target follower_record_control -j"$(nproc 2>/dev/null || echo 2)"
fi

echo "[openarm] starting right follower recorder: follower=${RIGHT_FOLLOWER_CAN} port=${RIGHT_RECORD_START_PORT}"
OPENARM_TELEOP_ARM_SIDE=right_arm \
OPENARM_FOLLOWER_CAN="${RIGHT_FOLLOWER_CAN}" \
OPENARM_RECORD_START_PORT="${RIGHT_RECORD_START_PORT}" \
OPENARM_RECORD_PATH="${RIGHT_RECORD_PATH}" \
OPENARM_POSITION_DUMP_PATH="${POSITION_DUMP_PATH}" \
OPENARM_REAL_FOLLOWER_RECORD_LOG="${RIGHT_LOG}" \
OPENARM_NO_RECORD_PANEL=1 \
OPENARM_SKIP_CAN_CONFIG=1 \
"${RUN_SINGLE}" &
RIGHT_PID=$!
wait_recorder_ready "${RIGHT_PID}" "${RIGHT_LOG}" "right"

echo "[openarm] starting left follower recorder: follower=${LEFT_FOLLOWER_CAN} port=${LEFT_RECORD_START_PORT}"
OPENARM_TELEOP_ARM_SIDE=left_arm \
OPENARM_FOLLOWER_CAN="${LEFT_FOLLOWER_CAN}" \
OPENARM_RECORD_START_PORT="${LEFT_RECORD_START_PORT}" \
OPENARM_RECORD_PATH="${LEFT_RECORD_PATH}" \
OPENARM_POSITION_DUMP_PATH="${POSITION_DUMP_PATH}" \
OPENARM_REAL_FOLLOWER_RECORD_LOG="${LEFT_LOG}" \
OPENARM_NO_RECORD_PANEL=1 \
OPENARM_SKIP_CAN_CONFIG=1 \
"${RUN_SINGLE}" &
LEFT_PID=$!
wait_recorder_ready "${LEFT_PID}" "${LEFT_LOG}" "left"

echo "[openarm] both follower-only recorders are running"
echo "[openarm] record name: ${RECORD_NAME}"
echo "[openarm] CAN: left=${LEFT_FOLLOWER_CAN} right=${RIGHT_FOLLOWER_CAN}"
echo "[openarm] gripper compat (VR): open=${OPENARM_GRIPPER_OPEN} closed=${OPENARM_GRIPPER_CLOSED} Kp=${OPENARM_GRIPPER_KP} Kd=${OPENARM_GRIPPER_KD}"
echo "[openarm] session dir: ${SESSION_DIR}"
echo "[openarm] right record: ${RIGHT_RECORD_PATH}"
echo "[openarm] left record : ${LEFT_RECORD_PATH}"
echo "[openarm] merged record: ${MERGED_RECORD_PATH}"
echo "[openarm] position snapshots: ${POSITION_DUMP_PATH}"
echo "[openarm] right log: ${RIGHT_LOG}"
echo "[openarm] left log : ${LEFT_LOG}"
echo "[openarm] opening one panel to start both follower recordings"
python "${SCRIPT_DIR}/record_start_panel_both.py" \
  --right-port "${RIGHT_RECORD_START_PORT}" \
  --left-port "${LEFT_RECORD_START_PORT}" \
  --right-log "${RIGHT_LOG}" \
  --left-log "${LEFT_LOG}" \
  --position-output "${POSITION_DUMP_PATH}" &
PANEL_PID=$!

echo "[openarm] close the dual record panel or press Ctrl+C here to stop both follower recorders"

while true; do
  if ! kill -0 "${PANEL_PID}" >/dev/null 2>&1; then
    echo "[openarm] dual record panel closed; stopping both follower recorders"
    exit 0
  fi
  if ! kill -0 "${RIGHT_PID}" >/dev/null 2>&1; then
    echo "[openarm][WARN] right follower recorder exited; stopping left"
    exit 1
  fi
  if ! kill -0 "${LEFT_PID}" >/dev/null 2>&1; then
    echo "[openarm][WARN] left follower recorder exited; stopping right"
    exit 1
  fi
  sleep 1
done
