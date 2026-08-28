#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REQUESTED_PANEL_PORT="${OPENARM_PANEL_PORT:-47910}"
LOG_PATH="${OPENARM_MUJOCO_LOG:-/tmp/openarm_cpp_mujoco.log}"

PANEL_PORT="$(
  python3 - "${REQUESTED_PANEL_PORT}" <<'PY'
import socket
import sys

start = int(sys.argv[1])
for port in range(start, start + 50):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", port))
    except OSError:
        sock.close()
        continue
    sock.close()
    print(port)
    break
else:
    raise SystemExit(f"no free UDP panel port found in [{start}, {start + 49}]")
PY
)"

if [ "${PANEL_PORT}" != "${REQUESTED_PANEL_PORT}" ]; then
  echo "[openarm] UDP panel port ${REQUESTED_PANEL_PORT} is busy, using ${PANEL_PORT}"
fi

export OPENARM_PANEL_PORT="${PANEL_PORT}"

"${SCRIPT_DIR}/run_cpp_mujoco_panel.sh" "$@" >"${LOG_PATH}" 2>&1 &
SIM_PID=$!

cleanup() {
  if kill -0 "${SIM_PID}" >/dev/null 2>&1; then
    kill "${SIM_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

sleep 1
echo "[openarm] C++ MuJoCo log: ${LOG_PATH}"
exec "${SCRIPT_DIR}/run_cpp_joint_panel.sh" --port "${PANEL_PORT}" --arms bimanual
