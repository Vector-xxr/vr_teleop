#!/usr/bin/env bash
# Build every native binary used by the maintained vr_control chains.
# This script configures/compiles only; it never touches CAN or starts a robot.
set -euo pipefail

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SELF_DIR}/.." && pwd)"
MAIN="${ROOT}/openarm_teleop/openarm_inplace"
VENDOR_ROOT="${MAIN}/skewer_topdown_2d_minimal/vendor"
VENDOR_INPLACE="${VENDOR_ROOT}/openarm_inplace"
VISION_TOOLS="${VENDOR_ROOT}/openarm_vision_grasp_yolo"

MAIN_BUILD="${MAIN}/build"
VENDOR_BUILD="${VENDOR_INPLACE}/build_current"
VISION_BUILD="${VISION_TOOLS}/build"

JOBS="$(nproc 2>/dev/null || echo 2)"
CLEAN=0
CHECK_ONLY=0
INSTALL_DEPS=0
BUILD_TYPE="Release"

usage() {
  cat <<EOF
Usage: $0 [options]

Builds native targets used by:
  - Quest VR teleop / recording
  - main-tree replay / import_task / leader-follower recording
  - left_job_orchestrator and tcp_forward_grasp

Options:
  --clean              remove only the three generated build directories first
  --check              do not build; verify required outputs and runtime links
  --install-deps       apt-install known Ubuntu build dependencies
  --jobs N             parallel build jobs (default: ${JOBS})
  --build-type TYPE    CMake build type (default: Release)
  -h, --help           show this help

This script does NOT configure CAN and does NOT move the robot.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean) CLEAN=1; shift ;;
    --check) CHECK_ONLY=1; shift ;;
    --install-deps) INSTALL_DEPS=1; shift ;;
    --jobs)
      [[ $# -ge 2 && "$2" =~ ^[1-9][0-9]*$ ]] || {
        echo "[build][ERROR] --jobs requires a positive integer" >&2
        exit 2
      }
      JOBS="$2"
      shift 2
      ;;
    --build-type)
      [[ $# -ge 2 && -n "$2" ]] || {
        echo "[build][ERROR] --build-type requires a value" >&2
        exit 2
      }
      BUILD_TYPE="$2"
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "[build][ERROR] unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

log() { printf '[build] %s\n' "$*"; }
die() { printf '[build][ERROR] %s\n' "$*" >&2; exit 1; }

for path in \
  "${MAIN}/CMakeLists.txt" \
  "${VENDOR_INPLACE}/CMakeLists.txt" \
  "${VISION_TOOLS}/CMakeLists.txt"
do
  [[ -f "${path}" ]] || die "missing source tree: ${path}"
done

if [[ "${INSTALL_DEPS}" -eq 1 ]]; then
  command -v apt-get >/dev/null 2>&1 ||
    die "--install-deps currently supports Debian/Ubuntu apt only"
  log "installing known C++ dependencies (does not install ROS/conda)"
  APT=(apt-get)
  if [[ "$(id -u)" -ne 0 ]]; then
    command -v sudo >/dev/null 2>&1 || die "sudo is required for --install-deps"
    APT=(sudo apt-get)
  fi
  "${APT[@]}" update
  KDL_PARSER_PKG=""
  for candidate in libkdl-parser-dev ros-humble-kdl-parser; do
    if apt-cache show "${candidate}" >/dev/null 2>&1; then
      KDL_PARSER_PKG="${candidate}"
      break
    fi
  done
  [[ -n "${KDL_PARSER_PKG}" ]] ||
    die "kdl_parser dev package unavailable; install ROS Humble first"
  "${APT[@]}" install -y \
    build-essential cmake pkg-config can-utils iproute2 \
    libeigen3-dev liborocos-kdl-dev "${KDL_PARSER_PKG}" \
    liburdfdom-dev liburdfdom-headers-dev libyaml-cpp-dev \
    libglfw3-dev libgl1-mesa-dev
fi

if [[ "$(id -u)" -eq 0 && "${CHECK_ONLY}" -eq 0 ]]; then
  die "do not compile as root; run this script as the normal robot user"
fi

for cmd in cmake c++ python3; do
  command -v "${cmd}" >/dev/null 2>&1 || die "required command not found: ${cmd}"
done

# ROS installations often provide kdl_parser's CMake package.
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  log "sourced /opt/ros/humble/setup.bash"
fi

choose_python() {
  if [[ -n "${OPENARM_PYTHON:-}" && -x "${OPENARM_PYTHON}" ]]; then
    printf '%s\n' "${OPENARM_PYTHON}"
    return
  fi
  local candidate
  for candidate in \
    "${HOME}/miniconda3/envs/openarm_teleop1/bin/python" \
    "/home/knowin/miniconda3/envs/openarm_teleop1/bin/python"
  do
    if [[ -x "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return
    fi
  done
  command -v python3
}

PYTHON_BIN="$(choose_python)"
log "repo=${ROOT}"
log "python=${PYTHON_BIN}"
log "jobs=${JOBS} build_type=${BUILD_TYPE}"

find_mujoco_prefix() {
  "${PYTHON_BIN}" - <<'PY'
from pathlib import Path
try:
    import mujoco
except Exception:
    raise SystemExit(1)
p = Path(mujoco.__file__).resolve().parent
if not (p / "include" / "mujoco" / "mujoco.h").is_file():
    raise SystemExit(2)
print(p)
PY
}

verify_binary() {
  local path="$1"
  [[ -x "${path}" ]] || {
    echo "[check][MISS] ${path}" >&2
    return 1
  }
  if command -v ldd >/dev/null 2>&1; then
    if ldd "${path}" 2>/dev/null | grep -F 'not found' >/dev/null 2>&1; then
      echo "[check][BROKEN-LINK] ${path}" >&2
      ldd "${path}" | grep -F 'not found' >&2 || true
      return 1
    fi
  fi
  echo "[check][OK] ${path}"
}

check_outputs() {
  local failures=0
  # Main tree: VR, import_task, replay and record chains.
  for bin in \
    quest_cartesian_control \
    dual_replay_control \
    follower_record_control \
    unilateral_control \
    bilateral_control
  do
    verify_binary "${MAIN_BUILD}/${bin}" || failures=$((failures + 1))
  done

  # Vendor dual replay is intentionally separate: it contains tcp_vel approach.
  verify_binary "${VENDOR_BUILD}/dual_replay_control" ||
    failures=$((failures + 1))
  verify_binary "${VISION_BUILD}/kdl_fk_ik_tool" ||
    failures=$((failures + 1))

  "${PYTHON_BIN}" - \
    "${ROOT}/left_job_orchestrator/service.py" \
    "${ROOT}/left_job_orchestrator/client.py" \
    "${ROOT}/left_job_orchestrator/protocol.py" \
    "${ROOT}/left_job_orchestrator/test_socket.py" \
    "${ROOT}/tcp_forward_grasp/flow.py" \
    "${ROOT}/tcp_forward_grasp/flow_loader.py" <<'PY' ||
import ast
import importlib
import pathlib
import sys
for module in ("mujoco", "numpy", "cv2"):
    importlib.import_module(module)
    print(f"[check][OK] Python module: {module}")
for raw in sys.argv[1:]:
    path = pathlib.Path(raw)
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    print(f"[check][OK] Python syntax: {path}")
PY
    failures=$((failures + 1))

  if [[ "${failures}" -ne 0 ]]; then
    die "verification failed (${failures} missing/broken item(s))"
  fi
  log "verification passed; no hardware was accessed"
}

if [[ "${CHECK_ONLY}" -eq 1 ]]; then
  check_outputs
  exit 0
fi

MUJOCO_PREFIX="$(find_mujoco_prefix)" ||
  die "MuJoCo C library not found in ${PYTHON_BIN}. Install/update conda env openarm_teleop1 or set OPENARM_PYTHON."
log "mujoco_prefix=${MUJOCO_PREFIX}"

if [[ "${CLEAN}" -eq 1 ]]; then
  log "removing generated build directories"
  rm -rf "${MAIN_BUILD}" "${VENDOR_BUILD}" "${VISION_BUILD}"
fi

log "1/3 configure main openarm_inplace"
cmake -S "${MAIN}" -B "${MAIN_BUILD}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DMUJOCO_PYTHON_PREFIX="${MUJOCO_PREFIX}"

log "1/3 build main control targets"
cmake --build "${MAIN_BUILD}" --parallel "${JOBS}" --target \
  quest_cartesian_control \
  dual_replay_control \
  follower_record_control \
  unilateral_control \
  bilateral_control \
  gripper_jog \
  gripper_teleop_test \
  gripper_only_teleop_test

log "2/3 configure vendor openarm_inplace (tcp_vel visual approach)"
cmake -S "${VENDOR_INPLACE}" -B "${VENDOR_BUILD}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DMUJOCO_PYTHON_PREFIX="${MUJOCO_PREFIX}"

log "2/3 build vendor dual_replay_control"
cmake --build "${VENDOR_BUILD}" --parallel "${JOBS}" --target dual_replay_control

log "3/3 configure/build KDL FK/IK tool"
cmake -S "${VISION_TOOLS}" -B "${VISION_BUILD}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build "${VISION_BUILD}" --parallel "${JOBS}" --target kdl_fk_ik_tool

check_outputs

cat <<EOF

[build] DONE
  main:   ${MAIN_BUILD}
  vendor: ${VENDOR_BUILD}  (used by visual grasp; do not replace with main)
  kdl:    ${VISION_BUILD}/kdl_fk_ik_tool

Next safe step:
  ${ROOT}/migration_build/build_all.sh --check

Only after check passes and the robot area is safe:
  cd ${ROOT}/left_job_orchestrator && ./run_service.sh
EOF
