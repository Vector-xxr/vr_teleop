#!/usr/bin/env bash
# Shared env for KER launch scripts. Source from sibling run_*.sh.
# Override anything via environment before sourcing.

: "${KER_ROOT:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"

# Prefer: OPENARM_PYTHON → conda env openarm_teleop1 → python3
if [[ -z "${OPENARM_PYTHON:-}" ]]; then
  if [[ -x "${HOME}/miniconda3/envs/openarm_teleop1/bin/python" ]]; then
    OPENARM_PYTHON="${HOME}/miniconda3/envs/openarm_teleop1/bin/python"
  elif [[ -x "${HOME}/anaconda3/envs/openarm_teleop1/bin/python" ]]; then
    OPENARM_PYTHON="${HOME}/anaconda3/envs/openarm_teleop1/bin/python"
  elif command -v python3 >/dev/null 2>&1; then
    OPENARM_PYTHON="$(command -v python3)"
  else
    OPENARM_PYTHON="python3"
  fi
fi
export OPENARM_PYTHON

# Offline pyusb (vendored when pip is unreachable)
if [[ -d "${KER_ROOT}/.deps/pyusb/usb" ]]; then
  export PYTHONPATH="${KER_ROOT}/.deps/pyusb${PYTHONPATH:+:${PYTHONPATH}}"
fi

# CAN — prefer this lab's udev names (follower_*), else raw can0/can1.
# Override: export OPENARM_RIGHT_CAN=... OPENARM_LEFT_CAN=... before sourcing.
if [[ -z "${OPENARM_RIGHT_CAN:-}" ]]; then
  if [[ -e /sys/class/net/follower_right ]]; then
    OPENARM_RIGHT_CAN=follower_right
  else
    OPENARM_RIGHT_CAN=can0
  fi
fi
if [[ -z "${OPENARM_LEFT_CAN:-}" ]]; then
  if [[ -e /sys/class/net/follower_left ]]; then
    OPENARM_LEFT_CAN=follower_left
  else
    OPENARM_LEFT_CAN=can1
  fi
fi
export OPENARM_RIGHT_CAN OPENARM_LEFT_CAN

# Optional: where openarm_can python package sources live (for pip install -e)
if [[ -z "${OPENARM_CAN_PYTHON:-}" ]]; then
  for cand in \
    "${KER_ROOT}/.deps/openarm_can/python" \
    "${KER_ROOT}/../openarm_teleop/openarm_can/python" \
    "${HOME}/openarm_teleop1/openarm_teleop/openarm_can/python"
  do
    if [[ -f "${cand}/pyproject.toml" ]] || [[ -f "${cand}/setup.py" ]]; then
      OPENARM_CAN_PYTHON="${cand}"
      break
    fi
  done
fi
export OPENARM_CAN_PYTHON="${OPENARM_CAN_PYTHON:-}"

# MuJoCo scene (sim only)
if [[ -z "${OPENARM_MUJOCO_SCENE:-}" ]]; then
  for cand in \
    "${KER_ROOT}/assets/scene_table.xml" \
    "${KER_ROOT}/../openarm_teleop/openarm_mujoco/v1/scene_table.xml" \
    "${HOME}/openarm_teleop1/openarm_teleop/openarm_mujoco/v1/scene_table.xml"
  do
    if [[ -f "${cand}" ]]; then
      OPENARM_MUJOCO_SCENE="${cand}"
      break
    fi
  done
fi
export OPENARM_MUJOCO_SCENE="${OPENARM_MUJOCO_SCENE:-}"

: "${DISPLAY:=:0}"
export DISPLAY

# Do NOT hardcode production passwords in git. Set on the machine if needed:
#   export OPENARM_SUDO_PASSWORD='...'
