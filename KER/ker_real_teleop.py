#!/usr/bin/env python3
"""KER USB → real OpenArm followers (CAN), joint-space unilateral teleop.

Mapping (real hardware):
  - swap_j67=True (default)
  - negate_j6=False (default)
  - negate_j7=True (default)  # tip up/down after J6/J7 swap
  - invert_left_grip=True (default)  # left open/close same sense as right

Soft start (no hard snap):
  hold current pose → rate-limit command with FULL gains → free track after fixed time

Gripper grasp-hold (gripper only):
  follow KER grip while free; on close stall auto clamp with lag (τ≈Kp·lag);
  open KER past release threshold to let go. Arms stay continuous joint follow.

Recording (record == control):
  each tick writes the q actually sent via MIT (soft-start cmd or track target)
  CSV ~control-loop rate (default 200Hz), Quest column layout, no upsample

  UDP 127.0.0.1:47950  ← ker_record_panel.py
  → KER/recordings/sessions/<session>/<seg>/real_teleop_dual_merged.csv
  Closing the record panel sends quit and stops this teleop process.

Replay KER sessions with ./run_ker_replay.sh (200Hz, no interpolation).
"""

from __future__ import annotations

import argparse
import csv
import os
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
VENDOR = ROOT / "vendor"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(VENDOR))

from ker_joint_map import ker_angles_to_qpos8  # noqa: E402
from openarm_ker.ker_stream import KERStream, CMD_STANDBY, CMD_STREAM  # noqa: E402

try:
    import openarm_can as oa
except ImportError as e:
    print(
        "[ker-real] openarm_can not installed.\n"
        "  See KER/DEPLOY_PUBLIC.md § openarm_can\n"
        "  source KER/env.sh && pip install -e \"$OPENARM_CAN_PYTHON\"",
        file=sys.stderr,
    )
    raise SystemExit(1) from e


ARM_TYPES = [
    oa.MotorType.DM8009,
    oa.MotorType.DM8009,
    oa.MotorType.DM4340,
    oa.MotorType.DM4340,
    oa.MotorType.DM4310,
    oa.MotorType.DM4310,
    oa.MotorType.DM4310,
]
ARM_SEND = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
ARM_RECV = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
GRIP_TYPE = oa.MotorType.DM4310
GRIP_SEND = 0x08
GRIP_RECV = 0x18

DEFAULT_KP = [80.0, 80.0, 80.0, 80.0, 20.0, 24.0, 20.0, 8.0]
DEFAULT_KD = [2.0, 2.0, 2.0, 2.0, 0.25, 0.25, 0.25, 0.35]
# Only used for brief zero-Kp settle after enable / shutdown.
HOLD_KP = [0.0] * 8
HOLD_KD = [0.0] * 8

RECORD_ROOT = ROOT / "recordings" / "sessions"

# Gripper stroke after ker_joint_map (rad): open ≈ -1.05, closed ≈ +0.17
GRIP_OPEN_RAD = float(np.deg2rad(-60.0))
GRIP_CLOSED_RAD = float(np.deg2rad(10.0))
GRIP_CLOSE_DIR = 1.0  # positive motor = closing on this hardware

# Quest column layout (row rate = control loop; no upsample).
# q0..q7 = left, q8..q15 = right
QUEST_DUAL_HEADER = ["time"] + [
    col for i in range(16) for col in (f"q{i}", f"dq{i}", f"tau{i}")
]


class ActionRecorder:
    """Write dual-arm CSV: exactly the MIT q sent each tick (record == control)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._active = False
        self._writer = None
        self._fp = None
        self._path = None
        self._rows = 0
        self._t0_ns: int | None = None
        self._prev_q16: list[float] | None = None
        self._prev_t_s: float | None = None

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def start(self, session: str, seg: int | str = 1) -> Path:
        with self._lock:
            self._stop_unlocked()
            session = session.strip() or time.strftime("ker_%Y%m%d_%H%M%S")
            try:
                seg_name = f"{int(seg):02d}"
            except (TypeError, ValueError):
                seg_name = str(seg).strip() or "01"
            out_dir = RECORD_ROOT / session / seg_name
            out_dir.mkdir(parents=True, exist_ok=True)
            path = out_dir / "real_teleop_dual_merged.csv"
            self._fp = path.open("w", newline="", encoding="utf-8")
            self._writer = csv.writer(self._fp)
            self._writer.writerow(QUEST_DUAL_HEADER)
            self._path = path
            self._rows = 0
            self._t0_ns = None
            self._prev_q16 = None
            self._prev_t_s = None
            self._active = True
            print(f"[ker-real][record] START {path} (sent-MIT @ control rate, no upsample)")
            return path

    def stop(self) -> Path | None:
        with self._lock:
            path = self._path
            rows = self._rows
            self._stop_unlocked()
            if path is not None:
                print(f"[ker-real][record] STOP {path} rows={rows}")
            return path

    def _stop_unlocked(self):
        self._active = False
        if self._fp is not None:
            try:
                self._fp.flush()
                self._fp.close()
            except Exception:
                pass
        self._fp = None
        self._writer = None
        self._t0_ns = None
        self._prev_q16 = None
        self._prev_t_s = None

    def write_sent(self, ts_ns: int, q_l, q_r):
        """Record the joint command actually sent this tick (left then right)."""
        with self._lock:
            if not self._active or self._writer is None:
                return
            if self._t0_ns is None:
                self._t0_ns = int(ts_ns)
            t_s = (int(ts_ns) - self._t0_ns) * 1e-9
            q16 = [float(x) for x in q_l] + [float(x) for x in q_r]
            if len(q16) != 16:
                return
            # Finite-diff dq from adjacent real samples only (no upsample).
            if self._prev_q16 is None or self._prev_t_s is None:
                dq = [0.0] * 16
            else:
                dt = max(t_s - self._prev_t_s, 1e-6)
                dq = [(q16[i] - self._prev_q16[i]) / dt for i in range(16)]
            self._prev_q16 = q16
            self._prev_t_s = t_s
            row = [f"{t_s:.6f}"]
            for i in range(16):
                row.extend([f"{q16[i]:.9f}", f"{dq[i]:.9f}", "0.000000000"])
            self._writer.writerow(row)
            self._rows += 1
            if self._rows % 200 == 0:
                self._fp.flush()


def start_record_server(
    recorder: ActionRecorder,
    host: str,
    port: int,
    shutdown_event: threading.Event,
):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.settimeout(0.5)
    print(f"[ker-real] record UDP listening on {host}:{port}")

    def loop():
        while not shutdown_event.is_set():
            try:
                data, _addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            msg = data.decode("utf-8", errors="ignore").strip()
            if not msg:
                continue
            parts = msg.split(",")
            cmd = parts[0].strip().lower()
            if cmd == "start":
                session = parts[1].strip() if len(parts) > 1 else time.strftime("ker_%Y%m%d_%H%M%S")
                seg = parts[2].strip() if len(parts) > 2 else "1"
                recorder.start(session, seg)
            elif cmd == "stop":
                recorder.stop()
            elif cmd in ("quit", "shutdown"):
                print("[ker-real] record panel quit → shutting down teleop")
                recorder.stop()
                shutdown_event.set()
            else:
                print(f"[ker-real] unknown record cmd: {msg!r}")

    th = threading.Thread(target=loop, daemon=True)
    th.start()
    return sock


class GripperGraspAssist:
    """Gripper-only soft-stall + grasp-hold (arms stay continuous KER follow).

    Follow KER grip until closing stalls, then clamp with
    q_cmd = q_meas + close_dir * lag (τ ≈ Kp · lag). Open KER past the release
    threshold to exit hold.
    """

    def __init__(
        self,
        *,
        side: str,
        enabled: bool = True,
        lag: float = 0.22,
        lag_max: float = 0.30,
        stall_error: float = 0.04,
        stall_progress: float = 0.005,
        stall_time: float = 0.12,
        open_release: float = 0.12,
    ):
        self.side = side
        self.enabled = enabled
        self.lag_nom = max(0.0, lag)
        self.lag_max = max(self.lag_nom, lag_max)
        self.stall_error = max(0.0, stall_error)
        self.stall_progress = max(0.0, stall_progress)
        self.stall_time = max(0.0, stall_time)
        self.open_release = max(0.0, open_release)
        self.mode = "follow"  # follow | grasp
        self.lag = 0.0
        self.stall_timer = 0.0
        self.stall_q_ref: float | None = None

    @staticmethod
    def _clamp_stroke(q: float) -> float:
        lo = min(GRIP_OPEN_RAD, GRIP_CLOSED_RAD)
        hi = max(GRIP_OPEN_RAD, GRIP_CLOSED_RAD)
        return float(np.clip(q, lo, hi))

    def step(self, q_meas: float, q_ker: float, dt: float) -> float:
        if not self.enabled or dt <= 0.0:
            return float(q_ker)

        q_meas = float(q_meas)
        q_ker = float(q_ker)
        close_err = GRIP_CLOSE_DIR * (q_ker - q_meas)
        open_err = -close_err

        if self.mode == "grasp":
            mid = 0.5 * (GRIP_OPEN_RAD + GRIP_CLOSED_RAD)
            if open_err >= self.open_release or q_ker <= mid:
                print(
                    f"[ker-real] {self.side} gripper grasp-hold OFF "
                    f"(open intent ker={q_ker:+.3f} meas={q_meas:+.3f})"
                )
                self.mode = "follow"
                self.lag = 0.0
                self.stall_timer = 0.0
                self.stall_q_ref = None
                return self._clamp_stroke(q_ker)
            if close_err > 0.0:
                self.lag = min(self.lag_max, max(self.lag, self.lag_nom) + 0.5 * close_err)
            else:
                self.lag = max(self.lag_nom, self.lag)
            return self._clamp_stroke(q_meas + GRIP_CLOSE_DIR * self.lag)

        closing = close_err >= self.stall_error
        if closing:
            if self.stall_q_ref is None:
                self.stall_q_ref = q_meas
                self.stall_timer = 0.0
            progress = GRIP_CLOSE_DIR * (q_meas - self.stall_q_ref)
            if progress >= self.stall_progress:
                self.stall_q_ref = q_meas
                self.stall_timer = 0.0
            else:
                self.stall_timer += dt
            if self.stall_timer >= self.stall_time:
                seeded = float(
                    np.clip(max(close_err, self.lag_nom), self.lag_nom * 0.5, self.lag_max)
                )
                self.mode = "grasp"
                self.lag = seeded
                self.stall_timer = 0.0
                self.stall_q_ref = q_meas
                cmd = self._clamp_stroke(q_meas + GRIP_CLOSE_DIR * self.lag)
                print(
                    f"[ker-real] {self.side} gripper grasp-hold ON "
                    f"meas={q_meas:+.3f} lag={self.lag:.3f} cmd={cmd:+.3f} "
                    f"(open KER to release)"
                )
                return cmd
        else:
            self.stall_timer = 0.0
            self.stall_q_ref = None

        return self._clamp_stroke(q_ker)

    @property
    def holding(self) -> bool:
        return self.mode == "grasp"


def make_arm(iface: str) -> oa.OpenArm:
    arm = oa.OpenArm(iface, True)
    arm.init_arm_motors(ARM_TYPES, ARM_SEND, ARM_RECV)
    arm.init_gripper_motor(GRIP_TYPE, GRIP_SEND, GRIP_RECV)
    arm.set_callback_mode_all(oa.CallbackMode.STATE)
    arm.enable_all()
    q_hold = None
    for _ in range(40):
        arm.refresh_all()
        arm.recv_all(200)
        q_hold = read_q8(arm)
        send_mit(arm, q_hold, kp=HOLD_KP, kd=HOLD_KD)
        time.sleep(0.005)
    assert q_hold is not None
    # Brief full-gain hold at measured pose (no snap to KER yet).
    for _ in range(30):
        send_mit(arm, q_hold, kp=DEFAULT_KP, kd=DEFAULT_KD)
        time.sleep(0.01)
    return arm


def read_q8(arm: oa.OpenArm) -> np.ndarray:
    arm.refresh_all()
    arm.recv_all(200)
    q = [m.get_position() for m in arm.get_arm().get_motors()]
    g = [m.get_position() for m in arm.get_gripper().get_motors()]
    if len(q) != 7 or not g:
        raise RuntimeError(f"unexpected motor count arm={len(q)} grip={len(g)}")
    return np.asarray(q + [g[0]], dtype=np.float64)


def send_mit(arm: oa.OpenArm, q8: np.ndarray, kp=None, kd=None):
    kp = kp if kp is not None else DEFAULT_KP
    kd = kd if kd is not None else DEFAULT_KD
    arm_cmds = [
        oa.MITParam(float(kp[i]), float(kd[i]), float(q8[i]), 0.0, 0.0) for i in range(7)
    ]
    grip_cmds = [oa.MITParam(float(kp[7]), float(kd[7]), float(q8[7]), 0.0, 0.0)]
    arm.get_arm().mit_control_all(arm_cmds)
    arm.get_gripper().mit_control_all(grip_cmds)
    arm.recv_all(200)


class SlowChase:
    """Rate-limit the command; bite with FULL gains; free-track after fixed time.

    Phases:
      hold  — hold measured pose (full gains)
      chase — |Δcmd| ≤ step/tick toward KER (full gains)
      track — direct MIT to KER target (after chase_s; not gated on wrist err)
    """

    def __init__(
        self,
        *,
        hold_s: float,
        chase_s: float,
        step: float,
        grip_step: float,
    ):
        self.hold_s = hold_s
        self.chase_s = chase_s
        self.step = step
        self.grip_step = grip_step
        self.t0 = time.time()
        self.phase = "hold"
        self.cmd: np.ndarray | None = None
        self.done = False
        self.last_sent: np.ndarray | None = None

    def tick(self, arm, target: np.ndarray) -> str:
        if self.done:
            send_mit(arm, target)
            self.last_sent = np.asarray(target, dtype=np.float64).copy()
            return "track"

        now = time.time()
        elapsed = now - self.t0
        if self.cmd is None:
            self.cmd = read_q8(arm).copy()

        if self.phase == "hold":
            send_mit(arm, self.cmd, kp=DEFAULT_KP, kd=DEFAULT_KD)
            self.last_sent = self.cmd.copy()
            if elapsed >= self.hold_s:
                self.phase = "chase"
                self.t0 = now
                print("[ker-real] hold done → rate-limited chase (full gains; keep KER still)")
            return "hold"

        # Rate-limited command, full gains so motors track the slow cmd tightly.
        step8 = np.full(8, self.step, dtype=np.float64)
        step8[7] = self.grip_step
        err_cmd = target - self.cmd
        self.cmd = self.cmd + np.clip(err_cmd, -step8, step8)
        send_mit(arm, self.cmd, kp=DEFAULT_KP, kd=DEFAULT_KD)
        self.last_sent = self.cmd.copy()

        # Fixed-time unlock — do not wait for wrist err.
        if elapsed >= self.chase_s:
            self.done = True
            self.phase = "track"
            send_mit(arm, target)
            self.last_sent = np.asarray(target, dtype=np.float64).copy()
            return "track"
        return "chase"


def parse_args():
    p = argparse.ArgumentParser(description="KER → real OpenArm teleop")
    p.add_argument("--right-can", default=os.getenv("OPENARM_RIGHT_CAN", "can0"))
    p.add_argument("--left-can", default=os.getenv("OPENARM_LEFT_CAN", "can1"))
    p.add_argument("--hz", type=float, default=200.0)
    p.add_argument("--swap-j67", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument(
        "--negate-j6",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Negate OpenArm joint6 after swap (default: off).",
    )
    p.add_argument(
        "--negate-j7",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Negate OpenArm joint7 after swap = tip up/down (default: on).",
    )
    p.add_argument("--no-align", action="store_true")
    p.add_argument(
        "--align-hold",
        type=float,
        default=float(os.getenv("OPENARM_KER_ALIGN_HOLD", "0.8")),
        help="Seconds to hold current pose after enable (default 0.8).",
    )
    p.add_argument(
        "--align-chase",
        type=float,
        default=float(os.getenv("OPENARM_KER_ALIGN_CHASE", "4.0")),
        help="Seconds of rate-limited chase before free track (default 4).",
    )
    p.add_argument(
        "--align-step",
        type=float,
        default=0.004,
        help="Max rad per tick while chasing (~0.8 rad/s at 200Hz).",
    )
    p.add_argument(
        "--align-grip-step",
        type=float,
        default=0.008,
        help="Max gripper rad per tick while chasing.",
    )
    p.add_argument("--sides", choices=["both", "right", "left"], default="both")
    p.add_argument("--print-every", type=float, default=1.0)
    p.add_argument(
        "--align-trigger-gripper",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    p.add_argument(
        "--invert-left-grip",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Left gripper open/close same sense as right (default: on).",
    )
    p.add_argument(
        "--record-port",
        type=int,
        default=int(os.getenv("OPENARM_KER_RECORD_PORT", "47950")),
    )
    p.add_argument(
        "--record-host",
        default=os.getenv("OPENARM_KER_RECORD_HOST", "127.0.0.1"),
    )
    p.add_argument(
        "--panel",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Auto-launch ker_record_panel.py (default: on). Closing panel stops teleop.",
    )
    p.add_argument(
        "--grip-grasp-hold",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Gripper soft-stall grasp-hold assist (default: on).",
    )
    p.add_argument(
        "--grip-grasp-lag",
        type=float,
        default=float(os.getenv("OPENARM_KER_GRIP_GRASP_LAG", "0.22")),
        help="Grasp-hold lag rad (τ≈Kp·lag; default 0.22).",
    )
    p.add_argument(
        "--grip-grasp-lag-max",
        type=float,
        default=float(os.getenv("OPENARM_KER_GRIP_GRASP_LAG_MAX", "0.30")),
    )
    p.add_argument(
        "--grip-stall-error",
        type=float,
        default=0.04,
        help="Min KER→meas close error (rad) to arm soft-stall timer.",
    )
    p.add_argument(
        "--grip-stall-time",
        type=float,
        default=0.12,
        help="Seconds without close progress before grasp-hold.",
    )
    p.add_argument(
        "--grip-open-release",
        type=float,
        default=0.12,
        help="Open KER this many rad past meas to release grasp-hold.",
    )
    return p.parse_args()


def gripper_closed(q8: np.ndarray, side: str) -> bool:
    del side
    return float(q8[7]) > np.deg2rad(-5)


def maybe_launch_panel(args):
    if not args.panel:
        return None
    import subprocess

    env = os.environ.copy()
    env.setdefault("DISPLAY", ":0")
    panel = ROOT / "ker_record_panel.py"
    py = sys.executable
    proc = subprocess.Popen(
        [py, str(panel), "--host", args.record_host, "--port", str(args.record_port)],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    print(f"[ker-real] launched record panel → {panel.name} (pid={proc.pid})")
    print("[ker-real] closing the panel will stop this teleop process")
    return proc


def apply_gripper_assist(
    assist: GripperGraspAssist | None,
    arm,
    q_ker8: np.ndarray,
    dt: float,
) -> np.ndarray:
    """Override only gripper joint with grasp-hold assist; keep arm joints from KER."""
    out = np.asarray(q_ker8, dtype=np.float64).copy()
    if assist is None or arm is None:
        return out
    try:
        q_meas = read_q8(arm)
        out[7] = assist.step(float(q_meas[7]), float(q_ker8[7]), dt)
    except Exception:
        out[7] = float(q_ker8[7])
    return out


def main() -> int:
    args = parse_args()
    print(
        f"[ker-real] mapping swap_j67={args.swap_j67} "
        f"negate_j6={args.negate_j6} negate_j7={args.negate_j7} "
        f"invert_left_grip={args.invert_left_grip} "
        f"sides={args.sides} hz={args.hz}"
    )
    print(f"[ker-real] CAN right={args.right_can} left={args.left_can}")
    print(
        f"[ker-real] grip grasp-hold={args.grip_grasp_hold} "
        f"lag={args.grip_grasp_lag:.2f}/{args.grip_grasp_lag_max:.2f} "
        f"stall={args.grip_stall_time:.2f}s"
    )

    shutdown_event = threading.Event()
    recorder = ActionRecorder()
    start_record_server(recorder, args.record_host, args.record_port, shutdown_event)
    panel_proc = maybe_launch_panel(args)

    print("[ker-real] connecting KER ...")
    stream = KERStream()
    stream.connect()
    stream.send_command(CMD_STREAM)
    print(f"[ker-real] KER metadata: {stream.metadata}")

    right = left = None
    chase_r = chase_l = None
    grip_r = GripperGraspAssist(
        side="RIGHT",
        enabled=args.grip_grasp_hold,
        lag=args.grip_grasp_lag,
        lag_max=args.grip_grasp_lag_max,
        stall_error=args.grip_stall_error,
        stall_time=args.grip_stall_time,
        open_release=args.grip_open_release,
    )
    grip_l = GripperGraspAssist(
        side="LEFT",
        enabled=args.grip_grasp_hold,
        lag=args.grip_grasp_lag,
        lag_max=args.grip_grasp_lag_max,
        stall_error=args.grip_stall_error,
        stall_time=args.grip_stall_time,
        open_release=args.grip_open_release,
    )
    try:
        if args.sides in ("both", "right"):
            print(f"[ker-real] enabling right on {args.right_can} ...")
            right = make_arm(args.right_can)
        if args.sides in ("both", "left"):
            print(f"[ker-real] enabling left on {args.left_can} ...")
            left = make_arm(args.left_can)

        if not args.no_align:
            print(
                f"[ker-real] soft start: hold {args.align_hold:.1f}s → "
                f"chase {args.align_chase:.1f}s @ step={args.align_step} "
                f"(full gains; then free track)"
            )
            if right is not None:
                chase_r = SlowChase(
                    hold_s=args.align_hold,
                    chase_s=args.align_chase,
                    step=args.align_step,
                    grip_step=args.align_grip_step,
                )
            if left is not None:
                chase_l = SlowChase(
                    hold_s=args.align_hold,
                    chase_s=args.align_chase,
                    step=args.align_step,
                    grip_step=args.align_grip_step,
                )

        dt = 1.0 / max(args.hz, 1.0)
        last_print = 0.0
        n = 0
        nan8 = np.full(8, np.nan)
        phase_r = phase_l = "track"
        sent_r = nan8.copy()
        sent_l = nan8.copy()

        while not shutdown_event.is_set():
            if panel_proc is not None and panel_proc.poll() is not None:
                print("[ker-real] record panel exited → shutting down teleop")
                shutdown_event.set()
                break

            t0 = time.time()
            pkt = stream.recv()
            if pkt is None or "angles" not in pkt:
                time.sleep(0.001)
                continue
            q_l, q_r = ker_angles_to_qpos8(
                pkt["angles"],
                swap_j67=args.swap_j67,
                negate_j6=args.negate_j6,
                negate_j7=args.negate_j7,
                invert_left_grip=args.invert_left_grip,
            )

            if right is not None:
                q_r_cmd = apply_gripper_assist(grip_r, right, q_r, dt)
                if chase_r is not None and not chase_r.done:
                    if args.align_trigger_gripper and not gripper_closed(q_r_cmd, "right"):
                        hold = read_q8(right)
                        send_mit(right, hold, kp=HOLD_KP, kd=HOLD_KD)
                        sent_r = hold
                        phase_r = "wait-grip"
                    else:
                        was_done = chase_r.done
                        phase_r = chase_r.tick(right, q_r_cmd)
                        sent_r = (
                            chase_r.last_sent
                            if chase_r.last_sent is not None
                            else q_r_cmd
                        )
                        if chase_r.done and not was_done:
                            print("[ker-real] RIGHT chase done → tracking")
                else:
                    send_mit(right, q_r_cmd)
                    sent_r = q_r_cmd
                    phase_r = "grasp" if grip_r.holding else "track"

            if left is not None:
                q_l_cmd = apply_gripper_assist(grip_l, left, q_l, dt)
                if chase_l is not None and not chase_l.done:
                    if args.align_trigger_gripper and not gripper_closed(q_l_cmd, "left"):
                        hold = read_q8(left)
                        send_mit(left, hold, kp=HOLD_KP, kd=HOLD_KD)
                        sent_l = hold
                        phase_l = "wait-grip"
                    else:
                        was_done = chase_l.done
                        phase_l = chase_l.tick(left, q_l_cmd)
                        sent_l = (
                            chase_l.last_sent
                            if chase_l.last_sent is not None
                            else q_l_cmd
                        )
                        if chase_l.done and not was_done:
                            print("[ker-real] LEFT chase done → tracking")
                else:
                    send_mit(left, q_l_cmd)
                    sent_l = q_l_cmd
                    phase_l = "grasp" if grip_l.holding else "track"

            if recorder.active:
                # Record exactly what was sent this tick (record == control).
                ql = sent_l if left is not None else nan8
                qr = sent_r if right is not None else nan8
                recorder.write_sent(time.time_ns(), ql, qr)

            n += 1
            now = time.time()
            if args.print_every > 0 and now - last_print >= args.print_every:
                err_r = err_l = float("nan")
                which_r = which_l = "?"
                grip_rv = float(sent_r[7]) if right is not None else float("nan")
                grip_lv = float(sent_l[7]) if left is not None else float("nan")
                try:
                    if right is not None:
                        d = np.abs(np.asarray(sent_r)[:7] - read_q8(right)[:7])
                        err_r = float(np.max(d))
                        which_r = f"j{int(np.argmax(d)) + 1}"
                    if left is not None:
                        d = np.abs(np.asarray(sent_l)[:7] - read_q8(left)[:7])
                        err_l = float(np.max(d))
                        which_l = f"j{int(np.argmax(d)) + 1}"
                except Exception:
                    pass
                rec = "REC" if recorder.active else "---"
                print(
                    f"[ker-real] n={n} [{rec}] "
                    f"R[{phase_r}] err={err_r:.3f}@{which_r} grip={grip_rv:+.3f} "
                    f"L[{phase_l}] err={err_l:.3f}@{which_l} grip={grip_lv:+.3f}"
                )
                last_print = now

            sleep = dt - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n[ker-real] stopping ...")
    finally:
        shutdown_event.set()
        recorder.stop()
        if panel_proc is not None and panel_proc.poll() is None:
            try:
                panel_proc.terminate()
            except Exception:
                pass
        for name, arm in (("right", right), ("left", left)):
            if arm is None:
                continue
            try:
                q = read_q8(arm)
                for _ in range(10):
                    send_mit(arm, q, kp=HOLD_KP, kd=HOLD_KD)
                    time.sleep(0.005)
                arm.disable_all()
                print(f"[ker-real] {name} disabled")
            except Exception as exc:
                print(f"[ker-real] {name} shutdown error: {exc}", file=sys.stderr)
        try:
            stream.send_command(CMD_STANDBY)
            stream.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
