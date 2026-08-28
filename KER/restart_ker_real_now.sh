#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${OPENARM_PYTHON:-/home/lan/miniconda3/envs/openarm_teleop1/bin/python}"
export DISPLAY="${DISPLAY:-:0}"
pkill -9 -f "${ROOT}/ker_real_teleop.py" 2>/dev/null || true
pkill -9 -f "${ROOT}/ker_record_panel.py" 2>/dev/null || true
sleep 1
: > /tmp/ker_real_teleop.log
nohup "${PY}" -u "${ROOT}/ker_real_teleop.py" \
  --right-can "${OPENARM_RIGHT_CAN:-can0}" \
  --left-can "${OPENARM_LEFT_CAN:-can1}" \
  --swap-j67 --no-negate-j6 --negate-j7 --invert-left-grip --panel \
  >> /tmp/ker_real_teleop.log 2>&1 &
echo "started $!"
sleep 2
pgrep -af "${ROOT}/ker_real_teleop.py" || echo "FAILED to stay up"
head -n 15 /tmp/ker_real_teleop.log
