#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${ROOT_DIR}/.." && pwd)"
TELEOP_ROOT="${REPO_ROOT}/openarm_teleop"
INPLACE_DIR="${TELEOP_ROOT}/openarm_inplace"
QUEST_DIR="${ROOT_DIR}"

# Align CAN + gripper env with record/replay (openarm_vr_compat_env.sh).
# shellcheck disable=SC1091
source "${INPLACE_DIR}/script/openarm_vr_compat_env.sh"

URDF="${OPENARM_TELEOP_URDF:-${TELEOP_ROOT}/openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf}"
FOLLOWER_CAN_LEFT="${OPENARM_FOLLOWER_CAN_LEFT}"
FOLLOWER_CAN_RIGHT="${OPENARM_FOLLOWER_CAN_RIGHT}"
QUEST_PORT_LEFT="${OPENARM_QUEST_PORT_LEFT:-47912}"
QUEST_PORT_RIGHT="${OPENARM_QUEST_PORT_RIGHT:-47911}"
HTTP_PORT="${OPENARM_QUEST_HTTP_PORT:-8443}"
QUEST_BIMANUAL=1

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

quest_activate_conda
quest_stop_stale_control
quest_build_control

quest_configure_can_interfaces "${FOLLOWER_CAN_LEFT}" "${FOLLOWER_CAN_RIGHT}"
quest_check_can_interface "${FOLLOWER_CAN_LEFT}"
quest_check_can_interface "${FOLLOWER_CAN_RIGHT}"

quest_start_bridge
quest_print_quest_url
quest_prepare_hm_logs "${INPLACE_DIR}"

echo "[quest] Launching real-hardware Quest Cartesian (bimanual)..."
echo "  URDF=${URDF}"
echo "  left_can=${FOLLOWER_CAN_LEFT}  right_can=${FOLLOWER_CAN_RIGHT}"
echo "  left_udp=${QUEST_PORT_LEFT}  right_udp=${QUEST_PORT_RIGHT}"
echo "  Stop: Ctrl+C in this terminal, or 退出 on the action-record panel"
echo "        (Kd damping ~${OPENARM_QUEST_DAMP_SEC:-6}s then motors disable)"

EXTRA_ARGS=()
if [[ "${OPENARM_QUEST_AUTO_HOME:-0}" == "1" ]]; then
  EXTRA_ARGS+=(--auto-home)
fi

# ---------------------------------------------------------------------------
# Pre-teleop startup motion defaults (override any of these via env).
# Change the defaults HERE when you want a new "power-on → ready" posture.
#   OPENARM_QUEST_STARTUP_MOVE            1=on, 0=keep power-on pose
#   OPENARM_QUEST_STARTUP_FILE            dual merged CSV or pose file; empty=TABLE_READY
#   OPENARM_QUEST_STARTUP_SEC             duration when file has no times / TABLE_READY (s)
#   OPENARM_QUEST_STARTUP_TRANSITION_SEC  quintic current→first CSV frame (s); 0=adaptive
#   OPENARM_QUEST_STARTUP_HOLD_SEC        hold after startup before clutch (read in C++, default 0.6)
# ---------------------------------------------------------------------------
: "${OPENARM_QUEST_STARTUP_MOVE:=1}"
: "${OPENARM_QUEST_STARTUP_SEC:=4}"
: "${OPENARM_QUEST_STARTUP_TRANSITION_SEC:=5}"
: "${OPENARM_QUEST_STARTUP_FILE:=openarm_teleop/openarm_inplace/recordings/tasks/steady_action_on_table.csv}"

if [[ "${OPENARM_QUEST_STARTUP_MOVE}" == "1" ]]; then
  EXTRA_ARGS+=(--startup-move)
fi
if [[ -n "${OPENARM_QUEST_STARTUP_FILE}" ]]; then
  _startup_file="${OPENARM_QUEST_STARTUP_FILE}"
  if [[ "${_startup_file}" != /* ]]; then
    if [[ -f "${REPO_ROOT}/${_startup_file}" ]]; then
      _startup_file="${REPO_ROOT}/${_startup_file}"
    elif [[ -f "${INPLACE_DIR}/${_startup_file}" ]]; then
      _startup_file="${INPLACE_DIR}/${_startup_file}"
    fi
  fi
  if [[ ! -f "${_startup_file}" ]]; then
    echo "[quest][ERROR] OPENARM_QUEST_STARTUP_FILE not found: ${OPENARM_QUEST_STARTUP_FILE}" >&2
    echo "[quest][HINT] Tried: ${OPENARM_QUEST_STARTUP_FILE}" >&2
    echo "[quest][HINT]         ${REPO_ROOT}/${OPENARM_QUEST_STARTUP_FILE}" >&2
    echo "[quest][HINT]         ${INPLACE_DIR}/${OPENARM_QUEST_STARTUP_FILE}" >&2
    echo "[quest][HINT] Set OPENARM_QUEST_STARTUP_FILE= to use TABLE_READY, or OPENARM_QUEST_STARTUP_MOVE=0 to skip." >&2
    exit 1
  fi
  echo "[quest] startup file: ${_startup_file}"
  EXTRA_ARGS+=(--startup-file "${_startup_file}")
else
  echo "[quest] startup target: TABLE_READY (no OPENARM_QUEST_STARTUP_FILE)"
fi
if [[ -n "${OPENARM_QUEST_STARTUP_SEC}" ]]; then
  EXTRA_ARGS+=(--startup-move-sec "${OPENARM_QUEST_STARTUP_SEC}")
fi
if [[ -n "${OPENARM_QUEST_STARTUP_TRANSITION_SEC}" ]]; then
  EXTRA_ARGS+=(--startup-transition-sec "${OPENARM_QUEST_STARTUP_TRANSITION_SEC}")
fi
# Extra CLI args pass-through, e.g. OPENARM_QUEST_EXTRA_ARGS="--teleop-mode human_mimic"
if [[ -n "${OPENARM_QUEST_EXTRA_ARGS:-}" ]]; then
  read -r -a _quest_extra_args <<< "${OPENARM_QUEST_EXTRA_ARGS}"
  EXTRA_ARGS+=("${_quest_extra_args[@]}")
fi

cd "${INPLACE_DIR}"
./build/quest_cartesian_control \
  --bimanual \
  --urdf "${URDF}" \
  --follower-can-left "${FOLLOWER_CAN_LEFT}" \
  --follower-can-right "${FOLLOWER_CAN_RIGHT}" \
  --quest-port-left "${QUEST_PORT_LEFT}" \
  --quest-port-right "${QUEST_PORT_RIGHT}" \
  "${EXTRA_ARGS[@]}" &
CONTROL_PID=$!
sleep 1
quest_start_ui_panels
wait "${CONTROL_PID}"
