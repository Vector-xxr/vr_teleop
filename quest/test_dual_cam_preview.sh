#!/usr/bin/env bash
# One-shot dual-cam preview self-test (run in a normal desktop terminal).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT}"

echo "=== devices ==="
ls -la /dev/video8 /dev/video16 2>&1 || true
echo "=== membership ==="
id
echo "=== busy ==="
fuser -v /dev/video8 /dev/video16 2>&1 || echo free
echo "=== DISPLAY ==="
echo "DISPLAY=${DISPLAY:-unset} XAUTHORITY=${XAUTHORITY:-unset}"
echo "=== opening preview 5s (ffplay windows should appear) ==="
timeout 5 python3 -u "${ROOT}/preview_dual_v4l_cams.py" --devices "${OPENARM_QUEST_RECORD_CAMS:-8,16}" 2>&1 || true
echo "=== done (exit of timeout is normal) ==="
