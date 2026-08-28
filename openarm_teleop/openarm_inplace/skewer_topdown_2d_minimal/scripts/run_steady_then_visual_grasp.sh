#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

PYTHON_BIN="${OPENARM_PYTHON:-/home/knowin/miniconda3/envs/openarm_teleop1/bin/python}"

STEADY_REPLAY="${STEADY_REPLAY:-/home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/recordings/tasks/steady_action_on_table.csv}"
OBSERVE_REPLAY="${OBSERVE_REPLAY:-${ROOT_DIR}/outputs/right_observe_record_20260727_154012/real_follower_right_02.csv}"

RIGHT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-follower_right}"
LEFT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-follower_left}"
SUDO_PASSWORD="${OPENARM_SUDO_PASSWORD:-knowin}"

INITIAL_TRANSITION_DURATION="${INITIAL_TRANSITION_DURATION:-3.0}"
STEADY_TO_OBSERVE_DURATION="${STEADY_TO_OBSERVE_DURATION:-6.0}"
AUTO_START_DELAY_S="${AUTO_START_DELAY_S:-1.0}"

cd "${ROOT_DIR}"

echo "[steady-visual] root=${ROOT_DIR}"
echo "[steady-visual] steady=${STEADY_REPLAY}"
echo "[steady-visual] observe=${OBSERVE_REPLAY}"
echo "[steady-visual] right_can=${RIGHT_FOLLOWER_CAN} left_can=${LEFT_FOLLOWER_CAN}"
echo "[steady-visual] flow: steady dual replay -> right observe transition -> auto visual XY/grasp/lift"

exec "${PYTHON_BIN}" scripts/align_once_move.py \
  --dual-replay \
  --execute \
  --steady-replay "${STEADY_REPLAY}" \
  --observe-replay "${OBSERVE_REPLAY}" \
  --right-follower-can "${RIGHT_FOLLOWER_CAN}" \
  --left-follower-can "${LEFT_FOLLOWER_CAN}" \
  --sudo-password "${SUDO_PASSWORD}" \
  --initial-transition-duration "${INITIAL_TRANSITION_DURATION}" \
  --pre-to-observe-duration "${STEADY_TO_OBSERVE_DURATION}" \
  --quick-grasp-on-d \
  --auto-once \
  --auto-start-delay-s "${AUTO_START_DELAY_S}" \
  --after-grasp-action lift-only \
  --no-quick-align-angle \
  --no-quick-verify-after-xy \
  --no-track-fallback-to-full \
  --no-preview \
  "$@"
