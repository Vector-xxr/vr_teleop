#!/usr/bin/env bash
# Smoke-test / run the KER (dora-openarm-data-collection) recording chain.
#
# Usage:
#   ./run_ker_record.sh ker-mujoco        # recommended first: real KER → MuJoCo viewer
#   ./run_ker_record.sh dummy-ker-mujoco  # no KER USB: dummy poses → MuJoCo
#   ./run_ker_record.sh ker-nocam         # real KER + CAN followers (after sim OK)
#   ./run_ker_record.sh dummy-nocam       # joints-only software smoke (no viewer)
#   ./run_ker_record.sh dummy | ker       # full stacks with cameras
#
# Prereqs (once):
#   curl -LsSf https://astral.sh/uv/install.sh | sh
#   export PATH="$HOME/.local/bin:$PATH"
#   # KER USB (M5Stack): lsusb | grep 303a
#   # udev: see nodes/dora-openarm-ker/README.md
#   # Real arms: configure can0/can1 CAN-FD (see README_RUN.md)
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${ROOT}/dora-openarm-data-collection"
MODE="${1:-ker-mujoco}"

export PATH="${HOME}/.local/bin:${PATH}"

if ! command -v uv >/dev/null 2>&1; then
  echo "[ker][ERROR] uv not found. Install first:"
  echo "  curl -LsSf https://astral.sh/uv/install.sh | sh"
  echo "  export PATH=\"\$HOME/.local/bin:\$PATH\""
  exit 1
fi

if ! command -v dora >/dev/null 2>&1; then
  echo "[ker] installing dora-rs-cli via uv tool..."
  uv tool install dora-rs-cli
  export PATH="${HOME}/.local/bin:${PATH}"
fi

cd "${REPO}"

case "${MODE}" in
  ker-mujoco|sim)
    FLOW="dataflow-ker-mujoco.yaml"
    echo "[ker] MODE=ker-mujoco (real KER USB → MuJoCo viewer, no CAN)"
    if ! lsusb 2>/dev/null | grep -qi '303a'; then
      echo "[ker][WARN] KER USB (303a) not seen — try: $0 dummy-ker-mujoco"
    fi
    echo "[ker] Move KER arms; MuJoCo window should follow."
    echo "[ker] UI http://127.0.0.1:8000 (optional record)"
    echo "[ker] Data: ${REPO}/ker_mujoco_data/joints_only/"
    ;;
  dummy-ker-mujoco)
    FLOW="dataflow-dummy-ker-mujoco.yaml"
    echo "[ker] MODE=dummy-ker-mujoco (fake leader → MuJoCo)"
    echo "[ker] UI http://127.0.0.1:8000"
    ;;
  dummy-nocam|nocam)
    FLOW="dataflow_dummy_nocam.yaml"
    echo "[ker] MODE=dummy-nocam (no cameras; joints only @ ~250Hz)"
    echo "[ker] UI http://127.0.0.1:8000  → Start! / Success!"
    echo "[ker] Data: ${REPO}/dataset_nocam/joints_only/"
    ;;
  dummy)
    FLOW="dataflow_dummy.yaml"
    echo "[ker] MODE=dummy (includes fake cameras)"
    echo "[ker] After start, open http://127.0.0.1:8000"
    echo "[ker] Data under: ${REPO}/dataset/"
    ;;
  ker-nocam)
    FLOW="dataflow-ker-nocam.yaml"
    echo "[ker] MODE=ker-nocam (real KER+CAN, no cameras)"
    if ! lsusb 2>/dev/null | grep -qi '303a'; then
      echo "[ker][WARN] M5Stack KER USB (vendor 303a) not seen. Plug it in."
    fi
    if ! ip -br link 2>/dev/null | grep -E '^can[0-9]+' | grep -q 'UP'; then
      echo "[ker][WARN] can0/can1 not UP. Need CAN-FD bringup before arms move."
    fi
    echo "[ker] UI: http://127.0.0.1:8000 → Start Arm → Start! → Success!"
    echo "[ker] Episodes: ${REPO}/ker_data/joints_only/episodes/"
    ;;
  ker)
    FLOW="dataflow-ker.yaml"
    echo "[ker] MODE=ker (real hardware + cameras)"
    if ! lsusb 2>/dev/null | grep -qi '303a'; then
      echo "[ker][WARN] M5Stack KER USB (vendor 303a) not seen. Plug it in."
    fi
    echo "[ker] Camera paths in dataflow-ker.yaml are placeholders (/dev/camera_*)."
    echo "[ker] UI: http://127.0.0.1:8000 → Start Arm → Start! → Success!"
    echo "[ker] Episodes: ${REPO}/ker_data/dataset/episodes/"
    ;;
  *)
    echo "Usage: $0 {ker-mujoco|dummy-ker-mujoco|ker-nocam|dummy-nocam|dummy|ker}"
    echo "  ker-mujoco       — try this first (KER → MuJoCo)"
    echo "  ker-nocam        — real arms after sim looks right"
    exit 2
    ;;
esac

echo "[ker] Building ${FLOW} ..."
uv run dora build "${FLOW}" --uv

echo "[ker] Running ${FLOW} (Ctrl+C to stop) ..."
exec uv run dora run "${FLOW}" --uv
