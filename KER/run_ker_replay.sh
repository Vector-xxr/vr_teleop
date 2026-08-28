#!/usr/bin/env bash
# Replay a KER teleop CSV at native ~200Hz (exact frames, no interpolation).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/env.sh"
PY="${OPENARM_PYTHON}"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <csv_or_segment_dir> [extra args...]" >&2
  exit 2
fi

pkill -9 -f 'ker_real_teleop.py' 2>/dev/null || true

exec "${PY}" -u "${ROOT}/ker_replay_csv.py" \
  --right-can "${OPENARM_RIGHT_CAN}" \
  --left-can "${OPENARM_LEFT_CAN}" \
  "$@"
