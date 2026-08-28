#!/usr/bin/env bash
# Read-only systemd preflight. Never configures CAN or starts robot processes.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VR_CONTROL="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${OPENARM_ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
PYTHON_BIN="${OPENARM_PYTHON:-}"
VENDOR_REPLAY="${VR_CONTROL}/openarm_teleop/openarm_inplace/skewer_topdown_2d_minimal/vendor/openarm_inplace/build_current/dual_replay_control"

if [[ -z "${OPENARM_SUDO_PASSWORD:-}" || "${OPENARM_SUDO_PASSWORD}" == "CHANGE_ME" ]]; then
  echo "[service-env][ERROR] OPENARM_SUDO_PASSWORD is not configured in env.systemd." >&2
  exit 1
fi

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "[service-env][ERROR] ROS setup is not readable: ${ROS_SETUP}" >&2
  echo "[service-env][HINT] Set OPENARM_ROS_SETUP in env.systemd." >&2
  exit 1
fi

if [[ -z "${PYTHON_BIN}" || ! -x "${PYTHON_BIN}" ]]; then
  echo "[service-env][ERROR] OPENARM_PYTHON is not an executable path." >&2
  exit 1
fi

# ROS setup scripts commonly reference unset variables. Its exported library
# path remains in this process and is inherited by build_all.sh and ldd.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

echo "[service-env] ROS setup loaded: ${ROS_SETUP}"
"${VR_CONTROL}/migration_build/build_all.sh" --check

if ! command -v ldd >/dev/null 2>&1; then
  echo "[service-env][ERROR] ldd is required for runtime-link validation." >&2
  exit 1
fi
if ldd "${VENDOR_REPLAY}" 2>/dev/null | grep -F 'not found' >/dev/null; then
  echo "[service-env][ERROR] vendor dual_replay_control has unresolved libraries:" >&2
  ldd "${VENDOR_REPLAY}" | grep -F 'not found' >&2 || true
  exit 1
fi

echo "[service-env] vendor ROS libraries resolve; preflight passed (no hardware accessed)"
