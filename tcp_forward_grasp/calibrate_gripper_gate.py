#!/usr/bin/env python3
"""Calibrate the fixed gripper-opening box (image pixels) and save to JSON.

Default motion: hold the current arm pose and only open the gripper to
begin.csv last q7 — no begin trajectory, no arm settle to begin-end.
Then open the camera UI for gate drawing / auto tip-line proposal.

Controls (camera UI):
  mouse drag     draw / redraw the inner opening box
  a              auto-propose from white tips (or green fallback)
  t              snap tip edge to tip-to-tip line
  arrows         nudge box 2px
  , / .          shrink / grow width
  [ / ]          move tip up / down
  ; / '          shrink / grow height from bottom
  s / Enter      save JSON (+ overlay snapshot)
  r              reset to last auto proposal
  q / Esc        quit without saving

Example:
  ./run_calibrate_gate.sh                 # open gripper in place, then UI
  ./run_calibrate_gate.sh --camera-only   # skip motion; gripper already open
  ./run_calibrate_gate.sh --play-begin    # full begin.csv (old behavior)
"""

from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from . import defaults, paths
from .camera_preview import grab_bgr, resize_preview
from .flow_loader import build_flow_args, load_flow_module
from .gripper_gate import (
    default_gate_path,
    draw_gate,
    gate_is_valid,
    load_gripper_gate,
    normalize_gate,
    propose_gate_from_green,
    propose_gate_white_fingers,
    read_begin_gripper,
    read_begin_last_q,
    save_gripper_gate,
    snap_gate_tip_to_white_fingers,
)
from .vision_align import target_center_px


WINDOW = "calibrate gripper gate"


class GateEditor:
    def __init__(self, gate: dict | None, image_size: tuple[int, int]):
        self.w, self.h = int(image_size[0]), int(image_size[1])
        self.gate = dict(gate) if gate_is_valid(gate) else None
        self.drag_start: tuple[int, int] | None = None
        self.drag_cur: tuple[int, int] | None = None
        self.proposal: dict | None = dict(gate) if gate_is_valid(gate) else None

    def set_gate(self, gate: dict | None) -> None:
        if gate_is_valid(gate):
            self.gate = normalize_gate(gate)
            self.proposal = dict(self.gate)

    def on_mouse(self, event, x, y, flags, _param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.drag_cur = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_cur = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
            x0, y0 = self.drag_start
            x1, y1 = x, y
            self.drag_start = None
            self.drag_cur = None
            if abs(x1 - x0) < 8 or abs(y1 - y0) < 8:
                return
            self.set_gate(
                {
                    "u_left": min(x0, x1),
                    "u_right": max(x0, x1),
                    "y_tip": min(y0, y1),
                    "y_bottom": max(y0, y1),
                    "image_width": self.w,
                    "image_height": self.h,
                    "source": "mouse_draw",
                }
            )

    def nudge(self, du: float = 0.0, dv: float = 0.0) -> None:
        if not self.gate:
            return
        g = self.gate
        for k in ("u_left", "u_right"):
            g[k] = float(np.clip(g[k] + du, 0, self.w - 1))
        for k in ("y_tip", "y_bottom"):
            g[k] = float(np.clip(g[k] + dv, 0, self.h - 1))
        self.set_gate(g)

    def adjust_width(self, d: float) -> None:
        if not self.gate:
            return
        mid = 0.5 * (self.gate["u_left"] + self.gate["u_right"])
        half = max(8.0, 0.5 * (self.gate["u_right"] - self.gate["u_left"]) + d)
        self.set_gate(
            {
                **self.gate,
                "u_left": mid - half,
                "u_right": mid + half,
                "source": "adjusted",
            }
        )

    def adjust_tip(self, d: float) -> None:
        if not self.gate:
            return
        tip = float(np.clip(self.gate["y_tip"] + d, 0, self.gate["y_bottom"] - 10))
        self.set_gate({**self.gate, "y_tip": tip, "source": "adjusted"})

    def adjust_bottom(self, d: float) -> None:
        if not self.gate:
            return
        bottom = float(np.clip(self.gate["y_bottom"] + d, self.gate["y_tip"] + 10, self.h - 1))
        self.set_gate({**self.gate, "y_bottom": bottom, "source": "adjusted"})


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "Open gripper to begin.csv last width in place (arm pose unchanged), "
            "then calibrate fixed gate box."
        )
    )
    p.add_argument(
        "--out",
        type=Path,
        default=default_gate_path(),
        help=f"output JSON (default: {default_gate_path()})",
    )
    p.add_argument(
        "--begin-csv",
        type=Path,
        default=paths.DEFAULT_BEGIN_CSV,
        help="begin.csv used to read final gripper open width / begin-end joints",
    )
    p.add_argument(
        "--camera-only",
        action="store_true",
        help="Do not move the arm; only open camera UI (gripper must already match begin)",
    )
    p.add_argument(
        "--play-begin",
        action="store_true",
        help="Play the full begin.csv trajectory (old behavior)",
    )
    p.add_argument(
        "--begin-transition-duration",
        type=float,
        default=1.0,
        help="Smooth seconds used if --play-begin; gripper-only open keeps arm fixed",
    )
    p.add_argument(
        "--open-hold-duration",
        type=float,
        default=0.8,
        help="Seconds to hold after gripper reaches begin open width",
    )
    p.add_argument(
        "--gripper-open-duration",
        type=float,
        default=1.2,
        help="Seconds to move gripper from current width to begin open",
    )
    p.add_argument("--camera-index", type=int, default=None)
    p.add_argument("--config", type=Path, default=None)
    p.add_argument("--preview-max-width", type=int, default=960)
    p.add_argument("--sudo-password", type=str, default=None)
    p.add_argument(
        "--note",
        type=str,
        default="",
    )
    return p


def _write_begin_end_open_hold(
    path: Path,
    *,
    q: list[float],
    gripper_open: float,
    hold_s: float,
    sample_period: float,
) -> Path:
    """Tiny CSV that holds begin-end joints with begin open gripper (not full begin)."""
    flow = load_flow_module()
    g = float(gripper_open)
    waypoints = [(list(q), g), (list(q), g)]
    durations = [max(0.2, float(hold_s))]
    flow.core.write_replay_csv_8d(path, waypoints, durations, float(sample_period))
    return path


def _open_gripper_to_begin(
    *,
    flow,
    server_proc,
    args,
    run_dir: Path,
    q: list[float],
    gripper_now: float,
    gripper_goal: float,
) -> float:
    """Hold arm joints, move only gripper to begin.csv last open width."""
    if abs(float(gripper_now) - float(gripper_goal)) < 1e-3:
        print(
            f"[calib-gate] gripper already at begin open {gripper_goal:+.6f}",
            flush=True,
        )
        return float(gripper_goal)
    replay_csv = run_dir / "open_gripper_to_begin.csv"
    waypoints = [
        (list(q), float(gripper_now)),
        (list(q), float(gripper_goal)),
        (list(q), float(gripper_goal)),
    ]
    durations = [
        float(getattr(args, "gripper_close_duration", 1.2)),
        float(getattr(args, "gripper_close_hold_duration", 0.8)),
    ]
    flow.core.write_replay_csv_8d(
        replay_csv,
        waypoints,
        durations,
        float(getattr(args, "replay_sample_period", 0.01)),
    )
    print(
        f"[calib-gate] opening gripper {gripper_now:+.6f} -> {gripper_goal:+.6f} (begin.csv last q7)",
        flush=True,
    )
    flow.core.play_full_replay_on_server(
        server_proc,
        replay_csv,
        args,
        label="calib-open-gripper",
        side="active",
        feedback_dir=run_dir / "feedback_dumps",
    )
    return float(gripper_goal)


def _arm_dump_side(args) -> str:
    """Map args.arm_side (e.g. left_arm) to feedback-dump side token (left/right)."""
    raw = str(getattr(args, "arm_side", "left_arm")).lower()
    if "right" in raw:
        return "right"
    return "left"


def _open_gripper_inplace(
    *,
    flow,
    args,
    run_dir: Path,
    begin_csv: Path,
    gripper_open: float,
    open_duration_s: float,
    hold_s: float,
) -> tuple[object, list[float], float]:
    """Start server, keep current arm joints, open gripper to begin.csv last q7."""
    g_goal = float(gripper_open)
    # Seed CSV is only for process boot; we never play it. Init-hold keeps current pose.
    q_seed, _g_seed = read_begin_last_q(begin_csv)
    seed_csv = run_dir / "calib_server_seed.csv"
    _write_begin_end_open_hold(
        seed_csv,
        q=q_seed,
        gripper_open=g_goal,
        hold_s=0.3,
        sample_period=float(getattr(args, "replay_sample_period", 0.01)),
    )
    print(
        f"[calib-gate] open gripper in place to {g_goal:+.6f} "
        f"(arm joints unchanged; not playing begin / not settling to begin-end)",
        flush=True,
    )
    server_proc = flow.calib.start_replay_server(seed_csv, args)
    flow.register_server_proc(server_proc)

    dump_path = run_dir / "feedback_dumps" / "calib_current_pose.txt"
    dump_path.parent.mkdir(parents=True, exist_ok=True)
    side = _arm_dump_side(args)
    fb = flow.core.request_arm_feedback_dump(
        server_proc,
        args,
        dump_path,
        label="calib-gate",
        side=side,
    )
    if fb is None:
        raise RuntimeError(
            "failed to read current arm feedback dump; "
            "cannot open gripper while holding current pose"
        )
    q_now, g_now = list(fb[0]), float(fb[1])
    print(
        f"[calib-gate] current feedback gripper={g_now:+.6f}; "
        f"holding {len(q_now)} arm joints",
        flush=True,
    )

    if abs(g_now - g_goal) < 1e-3:
        print(
            f"[calib-gate] gripper already at begin open {g_goal:+.6f}",
            flush=True,
        )
        args.gripper = float(g_goal)
        return server_proc, q_now, float(g_goal)

    open_csv = run_dir / "open_gripper_inplace.csv"
    waypoints = [
        (list(q_now), float(g_now)),
        (list(q_now), float(g_goal)),
        (list(q_now), float(g_goal)),
    ]
    durations = [max(0.2, float(open_duration_s)), max(0.2, float(hold_s))]
    flow.core.write_replay_csv_8d(
        open_csv,
        waypoints,
        durations,
        float(getattr(args, "replay_sample_period", 0.01)),
    )
    # First frame matches current commanded/measured joints → arm stays put.
    args.initial_transition_duration = min(
        float(getattr(args, "initial_transition_duration", 1.0)),
        0.35,
    )
    print(
        f"[calib-gate] gripper {g_now:+.6f} -> {g_goal:+.6f} "
        f"in {durations[0]:.2f}s (arm held)",
        flush=True,
    )
    flow.core.play_full_replay_on_server(
        server_proc,
        open_csv,
        args,
        label="calib-open-gripper-inplace",
        side="active",
        feedback_dir=run_dir / "feedback_dumps",
    )
    args.gripper = float(g_goal)
    return server_proc, q_now, float(g_goal)


def run_camera_ui(args, *, out_path: Path, note: str, gripper_open: float) -> int:
    flow = load_flow_module()
    servo = flow.servo
    editor: GateEditor | None = None
    existing = load_gripper_gate(out_path)

    print(
        "[calib-gate] Camera UI — draw the INNER opening between white fingers.\n"
        "[calib-gate] TIP edge = tip-to-tip grasp line (your red line), not past tips.\n"
        f"[calib-gate] gripper_open(begin)={gripper_open:+.6f}  save -> {out_path}",
        flush=True,
    )

    with servo.Camera(args) as cam:
        flow.core.flush_camera(cam, args)
        time.sleep(0.3)
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

        while True:
            frame = grab_bgr(cam)
            if frame is None:
                time.sleep(0.05)
                continue
            h, w = frame.shape[:2]
            if editor is None:
                seed = existing or propose_gate_from_green(args, w, h)
                auto = propose_gate_white_fingers(frame, args)
                if auto is not None:
                    seed = auto
                    print(
                        f"[calib-gate] auto white-finger proposal "
                        f"u=[{seed['u_left']:.0f},{seed['u_right']:.0f}] "
                        f"y=[{seed['y_tip']:.0f},{seed['y_bottom']:.0f}]",
                        flush=True,
                    )
                editor = GateEditor(seed, (w, h))
                cv2.setMouseCallback(WINDOW, editor.on_mouse)

            vis = frame.copy()
            try:
                cx, cy = target_center_px(args, w, h)
            except Exception:
                cx, cy = w * 0.5, h * 0.5
            cv2.drawMarker(
                vis,
                (int(cx), int(cy)),
                (0, 255, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=22,
                thickness=2,
            )
            if editor.drag_start and editor.drag_cur:
                x0, y0 = editor.drag_start
                x1, y1 = editor.drag_cur
                cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 128, 0), 2)
            if editor.gate:
                draw_gate(vis, editor.gate, label=f"gate {editor.gate.get('source', '')}")

            lines = [
                "drag=draw  a=auto  t=snap tip-line  []=tip  s=save  q=quit",
                f"gripper(begin)={gripper_open:+.4f}  out={out_path.name}",
                "TIP = tip-to-tip grasp face (not past tips, not at green)",
            ]
            if editor.gate:
                g = editor.gate
                lines.append(
                    f"u=[{g['u_left']:.0f},{g['u_right']:.0f}] "
                    f"y=[{g['y_tip']:.0f},{g['y_bottom']:.0f}]"
                )
            y = 28
            for line in lines:
                cv2.putText(
                    vis,
                    line,
                    (12, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                    lineType=cv2.LINE_AA,
                )
                y += 24

            show = resize_preview(vis, int(getattr(args, "preview_max_width", 960)))
            cv2.imshow(WINDOW, show)
            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), 27):
                print("[calib-gate] quit without saving", flush=True)
                break
            if key in (ord("s"), 13):
                if not editor.gate:
                    print("[calib-gate] no gate to save — draw a box first", flush=True)
                    continue
                gate = normalize_gate(
                    {
                        **editor.gate,
                        "image_width": w,
                        "image_height": h,
                        "camera_index": int(getattr(args, "camera_index", -1)),
                        "gripper_open_from_begin": float(gripper_open),
                        "begin_csv": str(paths.DEFAULT_BEGIN_CSV),
                    },
                    source="calibrated_fixed",
                )
                save_note = note or (
                    f"white gripper opening at begin.csv last q7={gripper_open:+.6f}; "
                    f"{datetime.now().isoformat(timespec='seconds')}"
                )
                save_gripper_gate(out_path, gate, note=save_note)
                snap = out_path.with_suffix(".png")
                draw_gate(vis, gate, label="SAVED FIXED")
                cv2.imwrite(str(snap), vis)
                print(f"[calib-gate] saved gate -> {out_path}", flush=True)
                print(f"[calib-gate] snapshot -> {snap}", flush=True)
                print(f"[calib-gate] recorded gripper_open_from_begin={gripper_open:+.6f}", flush=True)
                break
            if key == ord("a"):
                auto = propose_gate_white_fingers(frame, args)
                if auto is None:
                    auto = propose_gate_from_green(args, w, h)
                    print(
                        "[calib-gate] white-tip auto failed; "
                        "using green with tip ahead of crosshair",
                        flush=True,
                    )
                else:
                    print(
                        f"[calib-gate] white-tip auto y_tip={auto['y_tip']:.0f} "
                        f"(tips L/R={auto.get('tip_left_y')},{auto.get('tip_right_y')})",
                        flush=True,
                    )
                editor.set_gate(auto)
            if key == ord("t"):
                if not editor.gate:
                    print("[calib-gate] draw/auto a box first, then t to snap tip", flush=True)
                else:
                    snapped = snap_gate_tip_to_white_fingers(frame, editor.gate, args)
                    if snapped is None:
                        from .vision_align import target_center_px as _tcp

                        _tx, _ty = _tcp(args, w, h)
                        ahead = float(getattr(args, "calib_tip_ahead_of_green_px", 85.0))
                        g = dict(editor.gate)
                        g["y_tip"] = float(np.clip(_ty - ahead, 0, g["y_bottom"] - 10))
                        g["source"] = "snap_tip_ahead_green"
                        editor.set_gate(g)
                        print(
                            f"[calib-gate] white tip detect failed; "
                            f"set y_tip={g['y_tip']:.0f} (= green - {ahead:.0f}px)",
                            flush=True,
                        )
                    else:
                        editor.set_gate(snapped)
                        print(
                            f"[calib-gate] snapped y_tip to white tips -> {snapped['y_tip']:.0f}",
                            flush=True,
                        )
            if key == ord("r") and editor.proposal:
                editor.set_gate(editor.proposal)
            if key == 81 or key == ord("h"):
                editor.nudge(du=-2)
            if key == 83 or key == ord("l"):
                editor.nudge(du=+2)
            if key == 82 or key == ord("k"):
                editor.nudge(dv=-2)
            if key == 84 or key == ord("j"):
                editor.nudge(dv=+2)
            if key == ord(","):
                editor.adjust_width(-2)
            if key == ord("."):
                editor.adjust_width(+2)
            if key == ord("["):
                editor.adjust_tip(-2)
            if key == ord("]"):
                editor.adjust_tip(+2)
            if key == ord(";"):
                editor.adjust_bottom(-2)
            if key == ord("'"):
                editor.adjust_bottom(+2)

    cv2.destroyWindow(WINDOW)
    return 0


def main(argv: list[str] | None = None) -> int:
    cli = build_parser().parse_args(argv)
    begin_csv = Path(cli.begin_csv).expanduser().resolve()
    gripper_open = read_begin_gripper(begin_csv)
    _q_begin, g_check = read_begin_last_q(begin_csv)
    assert abs(g_check - gripper_open) < 1e-9

    print(
        f"[calib-gate] begin.csv last gripper (q7) = {gripper_open:+.6f}\n"
        f"[calib-gate] source: {begin_csv}",
        flush=True,
    )

    overrides = {
        "preview": True,
        "no_preview": False,
        "execute": not bool(cli.camera_only),
        "output_dir": paths.OUTPUT_ROOT,
        "observe_replay": begin_csv,
        "gripper_open": float(gripper_open),
        "gripper": float(gripper_open),
        "initial_transition_duration": float(cli.begin_transition_duration),
    }
    if cli.camera_index is not None:
        overrides["camera_index"] = int(cli.camera_index)
    if cli.config is not None:
        overrides["config"] = cli.config
    if cli.sudo_password is not None:
        overrides["sudo_password"] = cli.sudo_password

    args = build_flow_args(overrides=overrides, output_dir=paths.OUTPUT_ROOT)
    args.preview = True
    args.no_preview = False
    args.preview_max_width = int(cli.preview_max_width)
    args.gripper_open = float(gripper_open)
    args.gripper = float(gripper_open)
    args.initial_transition_duration = float(cli.begin_transition_duration)
    if cli.camera_index is not None:
        args.camera_index = int(cli.camera_index)
    elif not hasattr(args, "camera_index"):
        args.camera_index = defaults.CAMERA_INDEX

    out_path = Path(cli.out).expanduser().resolve()
    run_dir = paths.OUTPUT_ROOT / ("calib_gate_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    run_dir.mkdir(parents=True, exist_ok=True)

    if cli.camera_only:
        return run_camera_ui(
            args,
            out_path=out_path,
            note=str(cli.note),
            gripper_open=float(gripper_open),
        )

    flow = load_flow_module()
    flow.install_cleanup_handlers()
    server_proc = None
    try:
        if cli.play_begin:
            from .flow import start_and_play_begin

            print(
                "[calib-gate] --play-begin: replaying full begin.csv …",
                flush=True,
            )
            server_proc, q_end, _T_end, g_end = start_and_play_begin(
                args, run_dir, begin_csv
            )
            g_end = _open_gripper_to_begin(
                flow=flow,
                server_proc=server_proc,
                args=args,
                run_dir=run_dir,
                q=list(q_end),
                gripper_now=float(g_end),
                gripper_goal=float(gripper_open),
            )
        else:
            server_proc, q_end, g_end = _open_gripper_inplace(
                flow=flow,
                args=args,
                run_dir=run_dir,
                begin_csv=begin_csv,
                gripper_open=float(gripper_open),
                open_duration_s=float(cli.gripper_open_duration),
                hold_s=float(cli.open_hold_duration),
            )
        args.gripper = float(g_end)
        print(
            f"[calib-gate] arm held; gripper open={g_end:+.6f}; open camera UI",
            flush=True,
        )
        return run_camera_ui(
            args,
            out_path=out_path,
            note=str(cli.note),
            gripper_open=float(gripper_open),
        )
    finally:
        if server_proc is not None:
            flow.cleanup_running_processes("calib-gate finished")


if __name__ == "__main__":
    raise SystemExit(main())
