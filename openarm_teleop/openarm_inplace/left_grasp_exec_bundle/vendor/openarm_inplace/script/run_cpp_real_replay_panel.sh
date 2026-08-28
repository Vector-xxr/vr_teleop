#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  cat >&2 <<EOF
[openarm][ERROR] No graphical display is available, so the Tk replay selector panel cannot start.

Run this from a desktop terminal, or use X forwarding for SSH, for example:
  ssh -X knowin@<robot-pc>

For headless replay without the selector panel, run:
  cd "${PROJECT_DIR}"
  OPENARM_TORQUE_PANEL=0 script/run_cpp_real_replay_both.sh recordings/sessions/<session_name>

You can also pass a merged CSV or the whole recordings/sessions directory to run_cpp_real_replay_both.sh.
EOF
  exit 1
fi

echo "[openarm] Starting real replay panel (GUI)..."
echo "[openarm] A sudo password dialog may appear shortly for CAN config."
echo "[openarm] Close the panel window to stop dual_replay_control."
exec python3 "${SCRIPT_DIR}/real_replay_select_panel.py" --project-dir "${PROJECT_DIR}" "$@"
