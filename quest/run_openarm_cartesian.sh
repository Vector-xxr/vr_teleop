#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${ROOT_DIR}/.." && pwd)"
TELEOP_ROOT="${REPO_ROOT}/openarm_teleop"
INPLACE_DIR="${TELEOP_ROOT}/openarm_inplace"
QUEST_DIR="${ROOT_DIR}"

URDF="${OPENARM_TELEOP_URDF:-${TELEOP_ROOT}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
ARM_SIDE="${OPENARM_ARM_SIDE:-right_arm}"
FOLLOWER_CAN="${OPENARM_FOLLOWER_CAN:-can0}"
QUEST_PORT_RIGHT="${OPENARM_QUEST_PORT:-47911}"
HTTP_PORT="${OPENARM_QUEST_HTTP_PORT:-8443}"
QUEST_BIMANUAL=0

CONTROL_PID=""
BRIDGE_PID=""
OWNED_BRIDGE=0

# shellcheck disable=SC1091
source "${QUEST_DIR}/quest_real_common.sh"
trap quest_cleanup EXIT INT TERM

if [[ ! -f "${URDF}" ]]; then
  echo "[quest][ERROR] URDF not found: ${URDF}" >&2
  exit 1
fi

if [[ "${ARM_SIDE}" != "left_arm" && "${ARM_SIDE}" != "right_arm" ]]; then
  echo "[quest][ERROR] OPENARM_ARM_SIDE must be left_arm or right_arm" >&2
  exit 1
fi
if [[ "${ARM_SIDE}" == "left_arm" && "${FOLLOWER_CAN}" == "can0" && -z "${OPENARM_FOLLOWER_CAN:-}" ]]; then
  FOLLOWER_CAN="can1"
fi

quest_activate_conda
quest_stop_stale_control
quest_build_control

quest_configure_can_interfaces "${FOLLOWER_CAN}"
quest_check_can_interface "${FOLLOWER_CAN}"

quest_start_bridge
quest_print_quest_url
quest_prepare_hm_logs "${INPLACE_DIR}"

echo "[quest] Launching real-hardware Quest Cartesian (single arm)..."
echo "  URDF=${URDF}"
echo "  arm_side=${ARM_SIDE}"
echo "  follower_can=${FOLLOWER_CAN}"
echo "  quest_udp=${QUEST_PORT_RIGHT}"
echo "  Stop: Ctrl+C in this terminal"

EXTRA_ARGS=()
if [[ "${OPENARM_QUEST_AUTO_HOME:-0}" == "1" ]]; then
  EXTRA_ARGS+=(--auto-home)
fi
# Startup motion (smoothly raise arm before teleop). Default target = TABLE_READY.
if [[ "${OPENARM_QUEST_STARTUP_MOVE:-0}" == "1" ]]; then
  EXTRA_ARGS+=(--startup-move)
fi
if [[ -n "${OPENARM_QUEST_STARTUP_FILE:-}" ]]; then
  EXTRA_ARGS+=(--startup-file "${OPENARM_QUEST_STARTUP_FILE}")
fi
if [[ -n "${OPENARM_QUEST_STARTUP_SEC:-}" ]]; then
  EXTRA_ARGS+=(--startup-move-sec "${OPENARM_QUEST_STARTUP_SEC}")
fi
if [[ -n "${OPENARM_QUEST_STARTUP_TRANSITION_SEC:-}" ]]; then
  EXTRA_ARGS+=(--startup-transition-sec "${OPENARM_QUEST_STARTUP_TRANSITION_SEC}")
fi
# Extra CLI args pass-through, e.g. OPENARM_QUEST_EXTRA_ARGS="--teleop-mode human_mimic"
if [[ -n "${OPENARM_QUEST_EXTRA_ARGS:-}" ]]; then
  read -r -a _quest_extra_args <<< "${OPENARM_QUEST_EXTRA_ARGS}"
  EXTRA_ARGS+=("${_quest_extra_args[@]}")
fi

cd "${INPLACE_DIR}"
./build/quest_cartesian_control \
  --urdf "${URDF}" \
  --arm-side "${ARM_SIDE}" \
  --follower-can "${FOLLOWER_CAN}" \
  --quest-port "${QUEST_PORT_RIGHT}" \
  "${EXTRA_ARGS[@]}" &
CONTROL_PID=$!
wait "${CONTROL_PID}"
