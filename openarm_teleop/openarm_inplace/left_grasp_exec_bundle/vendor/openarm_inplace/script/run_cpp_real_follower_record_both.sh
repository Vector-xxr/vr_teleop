#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RUN_SINGLE="${SCRIPT_DIR}/run_cpp_real_follower_record.sh"

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
RIGHT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-follower_right}"
LEFT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-follower_left}"

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

echo "[openarm] starting right follower recorder: follower=${RIGHT_FOLLOWER_CAN} port=${RIGHT_RECORD_START_PORT}"
OPENARM_TELEOP_ARM_SIDE=right_arm \
OPENARM_FOLLOWER_CAN="${RIGHT_FOLLOWER_CAN}" \
OPENARM_RECORD_START_PORT="${RIGHT_RECORD_START_PORT}" \
OPENARM_RECORD_PATH="${RIGHT_RECORD_PATH}" \
OPENARM_POSITION_DUMP_PATH="${POSITION_DUMP_PATH}" \
OPENARM_REAL_FOLLOWER_RECORD_LOG="${RIGHT_LOG}" \
OPENARM_NO_RECORD_PANEL=1 \
"${RUN_SINGLE}" &
RIGHT_PID=$!

sleep 0.8

echo "[openarm] starting left follower recorder: follower=${LEFT_FOLLOWER_CAN} port=${LEFT_RECORD_START_PORT}"
OPENARM_TELEOP_ARM_SIDE=left_arm \
OPENARM_FOLLOWER_CAN="${LEFT_FOLLOWER_CAN}" \
OPENARM_RECORD_START_PORT="${LEFT_RECORD_START_PORT}" \
OPENARM_RECORD_PATH="${LEFT_RECORD_PATH}" \
OPENARM_POSITION_DUMP_PATH="${POSITION_DUMP_PATH}" \
OPENARM_REAL_FOLLOWER_RECORD_LOG="${LEFT_LOG}" \
OPENARM_NO_RECORD_PANEL=1 \
"${RUN_SINGLE}" &
LEFT_PID=$!

echo "[openarm] both follower-only recorders are running"
echo "[openarm] record name: ${RECORD_NAME}"
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
