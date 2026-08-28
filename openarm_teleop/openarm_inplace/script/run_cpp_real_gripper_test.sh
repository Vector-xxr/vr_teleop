#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SIDE="${OPENARM_GRIPPER_TEST_SIDE:-both}"
OPEN_POSITION="${OPENARM_GRIPPER_TEST_OPEN:-0.0}"
CLOSE_POSITION="${OPENARM_GRIPPER_TEST_CLOSE:-0.035}"
CYCLES="${OPENARM_GRIPPER_TEST_CYCLES:-3}"
CYCLE_DURATION="${OPENARM_GRIPPER_TEST_CYCLE_DURATION:-3.0}"
HZ="${OPENARM_GRIPPER_TEST_HZ:-200}"
EFFORT="${OPENARM_GRIPPER_TEST_EFFORT:-0.0}"
HOLD_POSE="${OPENARM_GRIPPER_TEST_HOLD_POSE:-${OPENARM_REPLAY_HOME_POSE:-${PROJECT_DIR}/recordings/sessions/test_zhengbei2/initial_p1.txt}}"
OUTPUT_DIR="${OPENARM_GRIPPER_TEST_OUTPUT_DIR:-}"
NO_RUN=0

usage() {
  cat <<EOF
Usage: $0 [options]

Generate a gripper-only merged replay CSV and run it through the normal
dual_replay_control real-hardware chain.

Options:
  --side left|right|both       Which gripper(s) to move. Default: ${SIDE}
  --open VALUE                 Open/reference position. Default: ${OPEN_POSITION}
  --close VALUE                Close/reference position. Default: ${CLOSE_POSITION}
  --cycles N                   Open-close-open cycles. Default: ${CYCLES}
  --cycle-duration SEC         Seconds per cycle. Default: ${CYCLE_DURATION}
  --hz HZ                      Generated CSV sample rate. Default: ${HZ}
  --effort VALUE               Gripper tau/feedforward column. Default: ${EFFORT}
  --hold-pose PATH             Position snapshot used to hold all non-test joints.
                               Default: ${HOLD_POSE}
  --output-dir DIR             Directory for generated CSV. Default: /tmp/openarm_gripper_test_*
  --no-run                     Only generate CSV; do not start real replay.
  -h, --help                   Show this help.

Environment variables with matching OPENARM_GRIPPER_TEST_* names are also supported.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --side)
      SIDE="$2"
      shift 2
      ;;
    --open)
      OPEN_POSITION="$2"
      shift 2
      ;;
    --close)
      CLOSE_POSITION="$2"
      shift 2
      ;;
    --cycles)
      CYCLES="$2"
      shift 2
      ;;
    --cycle-duration)
      CYCLE_DURATION="$2"
      shift 2
      ;;
    --hz)
      HZ="$2"
      shift 2
      ;;
    --effort)
      EFFORT="$2"
      shift 2
      ;;
    --hold-pose)
      HOLD_POSE="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --no-run)
      NO_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[openarm][ERROR] unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${SIDE}" in
  left|right|both) ;;
  *)
    echo "[openarm][ERROR] --side must be left, right, or both; got '${SIDE}'" >&2
    exit 2
    ;;
esac

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="$(mktemp -d /tmp/openarm_gripper_test.XXXXXX)"
else
  mkdir -p "${OUTPUT_DIR}"
fi

CSV_PATH="${OUTPUT_DIR}/real_teleop_dual_merged.csv"

python3 - \
  --hold-pose "${HOLD_POSE}" \
  --output "${CSV_PATH}" \
  --side "${SIDE}" \
  --open-position "${OPEN_POSITION}" \
  --close-position "${CLOSE_POSITION}" \
  --cycles "${CYCLES}" \
  --cycle-duration "${CYCLE_DURATION}" \
  --hz "${HZ}" \
  --effort "${EFFORT}" <<'PY'
import argparse
import csv
import math
import re
from pathlib import Path


def smoothstep(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5


def smoothstep_derivative(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return 30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4


def parse_follower_positions(line: str) -> list[float]:
    match = re.search(r"follower=\{([^}]*)\}", line)
    if not match:
        return []
    values: list[float] = []
    for item in match.group(1).split(","):
        if "=" not in item:
            continue
        values.append(float(item.split("=", 1)[1]))
    return values


def load_hold_pose(path: Path) -> tuple[list[float], list[float]]:
    left: list[float] | None = None
    right: list[float] | None = None
    if path.is_file():
        for line in path.read_text(errors="replace").splitlines():
            if "arm_side=left_arm" in line:
                values = parse_follower_positions(line)
                if len(values) == 8:
                    left = values
            elif "arm_side=right_arm" in line:
                values = parse_follower_positions(line)
                if len(values) == 8:
                    right = values
    if left is None or right is None:
        raise SystemExit(
            f"[openarm][ERROR] hold pose must contain left/right follower positions: {path}"
        )
    return left, right


def gripper_profile(
    t: float,
    cycle_duration: float,
    open_position: float,
    close_position: float,
) -> tuple[float, float]:
    phase = (t % cycle_duration) / cycle_duration
    half = 0.5 * cycle_duration
    delta = close_position - open_position
    if phase < 0.5:
        local = phase / 0.5
        value = open_position + delta * smoothstep(local)
        velocity = delta * smoothstep_derivative(local) / half
    else:
        local = (phase - 0.5) / 0.5
        value = close_position - delta * smoothstep(local)
        velocity = -delta * smoothstep_derivative(local) / half
    return value, velocity


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--hold-pose", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--side", choices=("left", "right", "both"), required=True)
    parser.add_argument("--open-position", type=float, required=True)
    parser.add_argument("--close-position", type=float, required=True)
    parser.add_argument("--cycles", type=int, required=True)
    parser.add_argument("--cycle-duration", type=float, required=True)
    parser.add_argument("--hz", type=float, required=True)
    parser.add_argument("--effort", type=float, required=True)
    args = parser.parse_args()

    if args.cycles <= 0:
        raise SystemExit("[openarm][ERROR] --cycles must be positive")
    if args.cycle_duration <= 0.0:
        raise SystemExit("[openarm][ERROR] --cycle-duration must be positive")
    if args.hz <= 0.0:
        raise SystemExit("[openarm][ERROR] --hz must be positive")

    left_hold, right_hold = load_hold_pose(args.hold_pose)
    total_duration = args.cycles * args.cycle_duration
    dt = 1.0 / args.hz
    steps = int(math.floor(total_duration / dt))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as handle:
        writer = csv.writer(handle)
        header = ["time"]
        for index in range(16):
            header.extend([f"q{index}", f"dq{index}", f"tau{index}"])
        writer.writerow(header)

        for step in range(steps + 1):
            t = min(step * dt, total_duration)
            left = list(left_hold)
            right = list(right_hold)
            left_dq = [0.0] * 8
            right_dq = [0.0] * 8
            left_tau = [0.0] * 8
            right_tau = [0.0] * 8
            grip_q, grip_dq = gripper_profile(
                t, args.cycle_duration, args.open_position, args.close_position
            )
            if args.side in ("left", "both"):
                left[7] = grip_q
                left_dq[7] = grip_dq
                left_tau[7] = args.effort
            if args.side in ("right", "both"):
                right[7] = grip_q
                right_dq[7] = grip_dq
                right_tau[7] = args.effort

            row = [f"{t:.6f}"]
            for q, dq, tau in zip(left, left_dq, left_tau):
                row.extend([f"{q:.9f}", f"{dq:.9f}", f"{tau:.9f}"])
            for q, dq, tau in zip(right, right_dq, right_tau):
                row.extend([f"{q:.9f}", f"{dq:.9f}", f"{tau:.9f}"])
            writer.writerow(row)

    print(f"[openarm] generated gripper test CSV: {args.output}")
    print(
        "[openarm] gripper test: "
        f"side={args.side} open={args.open_position} close={args.close_position} "
        f"cycles={args.cycles} duration={total_duration:.3f}s hz={args.hz}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
PY

echo "[openarm] hold pose: ${HOLD_POSE}"
echo "[openarm] generated replay: ${CSV_PATH}"

if [[ "${NO_RUN}" = "1" ]]; then
  echo "[openarm] --no-run set; not starting real replay"
  exit 0
fi

echo "[openarm] starting normal dual real replay chain for gripper test"
OPENARM_REPLAY_HOME_POSE="${HOLD_POSE}" \
"${SCRIPT_DIR}/run_cpp_real_replay_both.sh" "${CSV_PATH}"
