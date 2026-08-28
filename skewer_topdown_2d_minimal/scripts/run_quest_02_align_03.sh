#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKEWER_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
OPENARM_DIR="${SKEWER_DIR}/vendor/openarm_inplace"

SESSION_DIR="${OPENARM_QUEST_SESSION_DIR:-${SKEWER_DIR}/recordings/sessions/quest}"
ACTION1_CSV="${OPENARM_ACTION1_CSV:-${SESSION_DIR}/02/real_teleop_dual_merged.csv}"
ACTION2_CSV="${OPENARM_ACTION2_CSV:-${SESSION_DIR}/03/real_teleop_dual_merged.csv}"
BRIDGE_DIR="${OPENARM_ALIGN_BRIDGE_DIR:-${SESSION_DIR}/_generated_align_bridge}"
OBSERVE_REPLAY="${BRIDGE_DIR}/observe_from_action1_last_right.csv"
PLACE_Q_FILE="${BRIDGE_DIR}/place_q_from_action2_start_right.txt"

RIGHT_FOLLOWER_CAN="${OPENARM_RIGHT_FOLLOWER_CAN:-${OPENARM_FOLLOWER_CAN_RIGHT:-follower_right}}"
LEFT_FOLLOWER_CAN="${OPENARM_LEFT_FOLLOWER_CAN:-${OPENARM_FOLLOWER_CAN_LEFT:-follower_left}}"
RIGHT_CAMERA_BACKEND="${OPENARM_RIGHT_CAMERA_BACKEND:-opencv}"
RIGHT_CAMERA_INDEX="${OPENARM_RIGHT_CAMERA_INDEX:-16}"

COMMAND_PORT="${OPENARM_QUEST_ALIGN_COMMAND_PORT:-47920}"
STATUS_PORT="${OPENARM_QUEST_ALIGN_STATUS_PORT:-47921}"
SERVER_STARTUP_S="${OPENARM_QUEST_ALIGN_SERVER_STARTUP_S:-2.0}"
SETTLE_S="${OPENARM_QUEST_ALIGN_SETTLE_S:-0.5}"

MAX_MOVE_MM="${OPENARM_ALIGN_MAX_MOVE_MM:-180}"
XY_CORRECTION_SCALE="${OPENARM_ALIGN_XY_CORRECTION_SCALE:-1}"
GRASP_DESCENT_M="${OPENARM_GRASP_DESCENT_M:-0.04}"
GRASP_DESCENT_DURATION="${OPENARM_GRASP_DESCENT_DURATION:-5.0}"
PRE_CLOSE_HOLD_DURATION="${OPENARM_PRE_CLOSE_HOLD_DURATION:-0.6}"
GRIPPER_CLOSE_DURATION="${OPENARM_GRIPPER_CLOSE_DURATION:-1.2}"
GRIPPER_CLOSE_HOLD_DURATION="${OPENARM_GRIPPER_CLOSE_HOLD_DURATION:-1.2}"
POST_GRASP_LIFT_M="${OPENARM_POST_GRASP_LIFT_M:-0.10}"
POST_GRASP_LIFT_DURATION="${OPENARM_POST_GRASP_LIFT_DURATION:-3.0}"
PLACE_DURATION="${OPENARM_PLACE_DURATION:-3.0}"
GRIPPER_OPEN="${OPENARM_GRIPPER_OPEN:--0.5}"
GRIPPER_CLOSE="${OPENARM_GRIPPER_CLOSE:-0.1}"

QUICK_DETECT_PREVIEW_S="${OPENARM_QUICK_DETECT_PREVIEW_S:-0.25}"
BLACK_V_MAX="${OPENARM_BLACK_V_MAX:-80}"
MIN_AREA_PX="${OPENARM_MIN_AREA_PX:-250}"
TARGET_SELECTION="${OPENARM_TARGET_SELECTION:-skewer}"
TRACK_ROI_PX="${OPENARM_TRACK_ROI_PX:-90}"
TRACK_FALLBACK_TO_FULL="${OPENARM_TRACK_FALLBACK_TO_FULL:-0}"

mkdir -p "${BRIDGE_DIR}"

require_file() {
  local label="$1"
  local path="$2"
  if [ ! -f "${path}" ]; then
    echo "[quest-align][ERROR] ${label} not found: ${path}" >&2
    exit 1
  fi
}

require_file "action1 CSV" "${ACTION1_CSV}"
require_file "action2 CSV" "${ACTION2_CSV}"
require_file "align_once_move.py" "${SKEWER_DIR}/scripts/align_once_move.py"
require_file "run_cpp_real_replay_both.sh" "${OPENARM_DIR}/script/run_cpp_real_replay_both.sh"

python3 - "${ACTION1_CSV}" "${ACTION2_CSV}" "${OBSERVE_REPLAY}" "${PLACE_Q_FILE}" <<'PY'
import csv
import sys
from pathlib import Path

action1 = Path(sys.argv[1])
action2 = Path(sys.argv[2])
observe_out = Path(sys.argv[3])
place_q_out = Path(sys.argv[4])

def first_row(path: Path) -> dict[str, str]:
    with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row:
                return row
    raise RuntimeError(f"empty CSV: {path}")

def last_row(path: Path) -> dict[str, str]:
    last = None
    with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row:
                last = row
    if last is None:
        raise RuntimeError(f"empty CSV: {path}")
    return last

def right_q8(row: dict[str, str]) -> list[float]:
    return [float(row[f"q{i}"]) for i in range(8, 16)]

observe_q = right_q8(last_row(action1))
place_q = right_q8(first_row(action2))

observe_out.parent.mkdir(parents=True, exist_ok=True)
with observe_out.open("w", encoding="utf-8", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time", *[f"q{i}" for i in range(8)]])
    writer.writerow(["0.000000", *[f"{v:.9f}" for v in observe_q]])

place_q_out.write_text(",".join(f"{v:.9f}" for v in place_q[:7]) + "\n", encoding="utf-8")
print(f"[quest-align] observe replay from action1 last right pose: {observe_out}")
print(f"[quest-align] place-q from action2 first right arm pose: {place_q_out}")
print("[quest-align] place-q=" + ",".join(f"{v:.9f}" for v in place_q[:7]))
PY

PLACE_Q="$(tr -d '\n\r ' < "${PLACE_Q_FILE}")"
SERVER_PID=""

send_shutdown() {
  if [ -n "${SERVER_PID}" ] && kill -0 "${SERVER_PID}" >/dev/null 2>&1; then
    python3 - "${COMMAND_PORT}" <<'PY' || true
import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.sendto(b"shutdown", ("127.0.0.1", int(sys.argv[1])))
finally:
    sock.close()
PY
    wait "${SERVER_PID}" >/dev/null 2>&1 || true
  fi
}
trap send_shutdown EXIT INT TERM

play_and_wait() {
  local csv_path="$1"
  local label="$2"
  local side="${3:-}"
  python3 - "${COMMAND_PORT}" "${STATUS_PORT}" "${csv_path}" "${label}" "${side}" "${SETTLE_S}" <<'PY'
import csv
import socket
import sys
import time
from pathlib import Path

command_port = int(sys.argv[1])
status_port = int(sys.argv[2])
csv_path = Path(sys.argv[3]).resolve()
label = sys.argv[4]
side = sys.argv[5]
settle_s = float(sys.argv[6])

duration = 0.0
with csv_path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
    for row in csv.DictReader(f):
        if row and row.get("time"):
            duration = float(row["time"])

timeout_s = max(20.0, duration + 20.0 + settle_s)
status = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
status.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
status.bind(("127.0.0.1", status_port))
status.settimeout(0.2)

parts = ["play", str(csv_path), "0", label]
if side:
    parts.extend([side, "1.0"])
message = "|".join(parts)

sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    deadline = time.time() + timeout_s
    next_send = 0.0
    acknowledged = False
    while time.time() < deadline:
        now = time.time()
        if not acknowledged and now >= next_send:
            print(f"[quest-align] UDP {message} -> port {command_port}", flush=True)
            sender.sendto(message.encode("utf-8"), ("127.0.0.1", command_port))
            next_send = now + 1.0
        try:
            data, _ = status.recvfrom(4096)
        except socket.timeout:
            continue
        text = data.decode("utf-8", errors="replace")
        print(f"[quest-align][status] {text}", flush=True)
        if text.startswith(("accepted", "started")):
            acknowledged = True
        if text.startswith("finished"):
            time.sleep(settle_s)
            sys.exit(0)
        if text.startswith(("load_failed", "safety", "shutdown_done")):
            raise RuntimeError(text)
    raise TimeoutError(f"timeout waiting for {label} to finish after {timeout_s:.1f}s")
finally:
    sender.close()
    status.close()
PY
}

wait_for_server_ready() {
  python3 - "${STATUS_PORT}" "${OPENARM_QUEST_ALIGN_READY_TIMEOUT_S:-240}" <<'PY'
import socket
import sys
import time

status_port = int(sys.argv[1])
timeout_s = float(sys.argv[2])

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("127.0.0.1", status_port))
sock.settimeout(0.2)

deadline = time.time() + timeout_s
try:
    while time.time() < deadline:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        text = data.decode("utf-8", errors="replace")
        print(f"[quest-align][status] {text}", flush=True)
        if text.startswith(("ready", "motor_log")):
            sys.exit(0)
        if text.startswith(("load_failed", "safety", "shutdown_done")):
            raise RuntimeError(text)
    raise TimeoutError(f"timeout waiting for dual replay server ready after {timeout_s:.1f}s")
finally:
    sock.close()
PY
}

echo "[quest-align] package root: ${SKEWER_DIR}"
echo "[quest-align] action1: ${ACTION1_CSV}"
echo "[quest-align] action2: ${ACTION2_CSV}"
echo "[quest-align] CAN: left=${LEFT_FOLLOWER_CAN} right=${RIGHT_FOLLOWER_CAN}"
echo "[quest-align] right camera: ${RIGHT_CAMERA_BACKEND} /dev/video${RIGHT_CAMERA_INDEX}"

echo "[quest-align] step 1/3: start one dual replay server, then play action 02"
OPENARM_TORQUE_PANEL=0 \
OPENARM_LEFT_FOLLOWER_CAN="${LEFT_FOLLOWER_CAN}" \
OPENARM_RIGHT_FOLLOWER_CAN="${RIGHT_FOLLOWER_CAN}" \
"${OPENARM_DIR}/script/run_cpp_real_replay_both.sh" "${ACTION1_CSV}" \
  --command-port "${COMMAND_PORT}" \
  --status-port "${STATUS_PORT}" &
SERVER_PID=$!
if [ "${SERVER_STARTUP_S}" != "0" ]; then
  echo "[quest-align] waiting for dual replay server ready; timeout=${OPENARM_QUEST_ALIGN_READY_TIMEOUT_S:-240}s"
fi
if ! kill -0 "${SERVER_PID}" >/dev/null 2>&1; then
  echo "[quest-align][ERROR] dual replay server exited during startup" >&2
  wait "${SERVER_PID}" || true
  exit 1
fi
wait_for_server_ready
play_and_wait "${ACTION1_CSV}" "quest-02"

echo "[quest-align] step 2/3: align/grasp from action 02 final pose, then move right arm to action 03 start"
ALIGN_ARGS=(
  --observe-replay "${OBSERVE_REPLAY}"
  --quick-grasp-on-d
  --auto-once
  --exit-after-auto-once
  --auto-start-delay-s 1.0
  --auto-detect-retry-s 1.0
  --no-quick-align-angle
  --max-move-mm "${MAX_MOVE_MM}"
  --xy-correction-scale "${XY_CORRECTION_SCALE}"
  --grasp-descent-m "${GRASP_DESCENT_M}"
  --grasp-descent-duration "${GRASP_DESCENT_DURATION}"
  --pre-close-hold-duration "${PRE_CLOSE_HOLD_DURATION}"
  --gripper-close-duration "${GRIPPER_CLOSE_DURATION}"
  --gripper-close-hold-duration "${GRIPPER_CLOSE_HOLD_DURATION}"
  --post-grasp-lift-m "${POST_GRASP_LIFT_M}"
  --post-grasp-lift-duration "${POST_GRASP_LIFT_DURATION}"
  --place-q "${PLACE_Q}"
  --place-duration "${PLACE_DURATION}"
  --gripper-open "${GRIPPER_OPEN}"
  --gripper-close "${GRIPPER_CLOSE}"
  --quick-detect-preview-s "${QUICK_DETECT_PREVIEW_S}"
  --execute
  --external-replay-server
  --replay-command-format dual
  --replay-command-side right
  --replay-command-port "${COMMAND_PORT}"
  --no-auto-replay-command-port
  --follower-can "${RIGHT_FOLLOWER_CAN}"
  --follower-only
  --camera-backend "${RIGHT_CAMERA_BACKEND}"
  --camera-index "${RIGHT_CAMERA_INDEX}"
  --black-v-max "${BLACK_V_MAX}"
  --min-area-px "${MIN_AREA_PX}"
  --target-selection "${TARGET_SELECTION}"
  --track-roi-px "${TRACK_ROI_PX}"
)
if [ "${TRACK_FALLBACK_TO_FULL}" = "1" ]; then
  ALIGN_ARGS+=(--track-fallback-to-full)
else
  ALIGN_ARGS+=(--no-track-fallback-to-full)
fi
if [ -n "${OPENARM_SUDO_PASSWORD:-}" ]; then
  ALIGN_ARGS+=(--sudo-password "${OPENARM_SUDO_PASSWORD}")
fi
if [ -n "${OPENARM_ALIGN_EXTRA_ARGS:-}" ]; then
  read -r -a EXTRA_ALIGN_ARGS <<< "${OPENARM_ALIGN_EXTRA_ARGS}"
  ALIGN_ARGS+=("${EXTRA_ALIGN_ARGS[@]}")
fi
(
  cd "${SKEWER_DIR}"
  python scripts/align_once_move.py "${ALIGN_ARGS[@]}"
)

echo "[quest-align] step 3/3: play action 03 on the same dual replay server"
play_and_wait "${ACTION2_CSV}" "quest-03"

echo "[quest-align] done"
