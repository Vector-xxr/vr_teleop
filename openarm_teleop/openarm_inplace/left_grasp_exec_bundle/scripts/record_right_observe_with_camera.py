#!/usr/bin/env python3
"""
Start OpenArm follower recording while showing the wrist camera.

This script is intentionally separate from earlier skewer_2d experiments.
It launches:

  OPENARM_TELEOP_ARM_SIDE=right_arm/left_arm
  OPENARM_FOLLOWER_CAN=can0
  bash openarm_inplace/script/run_cpp_real_follower_record.sh

and opens a live camera preview in the same run.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from types import ModuleType

if "wayland" in os.environ.get("XDG_SESSION_TYPE", "").lower():
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import cv2


ROOT = Path(__file__).resolve().parents[1]
VR_ROOT = ROOT.parent
VENDOR_ROOT = ROOT / "vendor"
DEFAULT_OPENARM_INPLACE = (
    VENDOR_ROOT / "openarm_inplace"
    if (VENDOR_ROOT / "openarm_inplace").exists()
    else VR_ROOT / "openarm_inplace"
)
DEFAULT_TEMPLATE_ROOT = (
    VENDOR_ROOT / "template_library"
    if (VENDOR_ROOT / "template_library").exists()
    else VR_ROOT / "template_library"
)


def arm_label(arm_side: str) -> str:
    if arm_side == "left_arm":
        return "left"
    if arm_side == "right_arm":
        return "right"
    raise ValueError(f"Unsupported arm side: {arm_side}")


def load_module(module_name: str, path: Path) -> ModuleType:
    if not path.exists():
        raise FileNotFoundError(path)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class Camera:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.cap = None
        self.orbbec = None
        self.capture_module: ModuleType | None = None

    def __enter__(self):
        if self.args.camera_backend == "opencv":
            self.cap = cv2.VideoCapture(int(self.args.camera_index))
            if not self.cap.isOpened():
                raise RuntimeError(f"无法打开 OpenCV 相机 index={self.args.camera_index}")
            if self.args.camera_width > 0:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.args.camera_width))
            if self.args.camera_height > 0:
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.args.camera_height))
            if self.args.camera_fps > 0:
                self.cap.set(cv2.CAP_PROP_FPS, int(self.args.camera_fps))
        else:
            capture_path = (
                self.args.template_root.expanduser().resolve()
                / "rgbd_samurai_minimal"
                / "01_capture_rgbd.py"
            )
            self.capture_module = load_module("skewer_topdown_capture_rgbd", capture_path)
            self.orbbec = self.capture_module.OrbbecRgbdCamera(
                serial=str(self.args.orbbec_serial or "") or None
            )
            self.orbbec.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.orbbec is not None:
            self.orbbec.stop()
        if self.cap is not None:
            self.cap.release()

    def read(self):
        if self.cap is not None:
            ok, frame = self.cap.read()
            return frame if ok else None
        if self.orbbec is not None:
            assert self.capture_module is not None
            latest = None
            frames = self.orbbec.pipeline.wait_for_frames(int(self.args.frame_timeout_ms))
            if frames is not None:
                color_frame = frames.get_color_frame()
                if color_frame is not None:
                    latest = self.capture_module.frame_to_bgr(color_frame)
            for _ in range(max(0, int(self.args.flush_frames))):
                frames = self.orbbec.pipeline.wait_for_frames(1)
                if frames is None:
                    break
                color_frame = frames.get_color_frame()
                if color_frame is not None:
                    latest = self.capture_module.frame_to_bgr(color_frame)
            return latest
        return None


def probe_opencv_cameras(max_index: int, warmup_frames: int = 3) -> list[dict]:
    found = []
    for index in range(max(0, int(max_index)) + 1):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        opened = bool(cap.isOpened())
        width = height = fps = 0.0
        ok = False
        if opened:
            for _ in range(max(1, int(warmup_frames))):
                ok, frame = cap.read()
                if ok and frame is not None:
                    height, width = frame.shape[:2]
                    break
            fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
        cap.release()
        found.append(
            {
                "index": index,
                "opened": opened,
                "read_ok": bool(ok),
                "width": int(width),
                "height": int(height),
                "fps": fps,
            }
        )
    return found


def start_follower_record(args: argparse.Namespace, run_dir: Path) -> subprocess.Popen:
    openarm_inplace = args.openarm_inplace.expanduser().resolve()
    script = openarm_inplace / "script" / "run_cpp_real_follower_record.sh"
    if not script.exists():
        raise FileNotFoundError(script)

    record_path = args.record_path
    if record_path is None:
        record_path = run_dir / f"real_follower_{arm_label(args.arm_side)}.csv"
    else:
        record_path = record_path.expanduser().resolve()
    position_dump = args.position_dump
    if position_dump is None:
        position_dump = run_dir / "follower_position_snapshots.txt"
    else:
        position_dump = position_dump.expanduser().resolve()
    log_path = run_dir / "follower_record.log"
    record_path.parent.mkdir(parents=True, exist_ok=True)
    position_dump.parent.mkdir(parents=True, exist_ok=True)

    env = {
        **os.environ,
        "OPENARM_TELEOP_ARM_SIDE": str(args.arm_side),
        "OPENARM_FOLLOWER_CAN": str(args.follower_can),
        "OPENARM_RECORD_PATH": str(record_path),
        "OPENARM_POSITION_DUMP_PATH": str(position_dump),
        "OPENARM_RECORD_START_PORT": str(args.record_start_port),
        "OPENARM_REAL_FOLLOWER_RECORD_LOG": str(log_path),
    }
    if args.sudo_password:
        env["OPENARM_SUDO_PASSWORD"] = str(args.sudo_password)
    if args.no_record_panel:
        env["OPENARM_NO_RECORD_PANEL"] = "1"

    print(f"[record] starting {args.arm_side} follower record")
    print(f"[record] openarm_inplace={openarm_inplace}")
    print(f"[record] arm_side={args.arm_side}")
    print(f"[record] follower_can={args.follower_can}")
    print(f"[record] record_csv={record_path}")
    print(f"[record] position_dump={position_dump}")
    print(f"[record] log={log_path}")
    proc = subprocess.Popen(["bash", str(script)], cwd=str(openarm_inplace), env=env)
    time.sleep(1.5)
    if proc.poll() is not None:
        raise RuntimeError(f"follower record exited early with code {proc.returncode}; log={log_path}")
    return proc


def stop_process(proc: subprocess.Popen | None):
    if proc is None or proc.poll() is not None:
        return
    print("[record] stopping follower record process")
    proc.terminate()
    try:
        proc.wait(timeout=4.0)
    except subprocess.TimeoutExpired:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2.0)


def draw_overlay(frame, saved_count: int, run_dir: Path, arm_side: str):
    out = frame.copy()
    h, w = out.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.drawMarker(out, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 32, 2)
    cv2.putText(
        out,
        f"{arm_side} record panel + wrist camera | s save RGB | q quit",
        (18, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        lineType=cv2.LINE_AA,
    )
    cv2.putText(
        out,
        f"saved={saved_count} output={run_dir}",
        (18, h - 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 0),
        2,
        lineType=cv2.LINE_AA,
    )
    return out


def build_arg_parser() -> argparse.ArgumentParser:
    stamp = datetime.now().strftime("observe_record_%Y%m%d_%H%M%S")
    parser = argparse.ArgumentParser(description="Record one OpenArm follower while previewing wrist camera.")
    parser.add_argument("--openarm-inplace", type=Path, default=DEFAULT_OPENARM_INPLACE)
    parser.add_argument("--template-root", type=Path, default=DEFAULT_TEMPLATE_ROOT)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / stamp)
    parser.add_argument("--record-path", type=Path, default=None)
    parser.add_argument("--position-dump", type=Path, default=None)
    parser.add_argument("--arm-side", choices=("right_arm", "left_arm"), default="right_arm")
    parser.add_argument("--follower-can", default="can0")
    parser.add_argument("--record-start-port", type=int, default=47920)
    parser.add_argument("--sudo-password", default="")
    parser.add_argument("--no-record-panel", action="store_true", help="不打开 Save Current Positions 面板，仅等待外部 UDP 控制。")

    parser.add_argument("--camera-backend", choices=("opencv", "orbbec"), default="opencv")
    parser.add_argument("--camera-index", type=int, default=8)
    parser.add_argument("--list-cameras", action="store_true", help="只探测 OpenCV 相机 index，不启动录制。")
    parser.add_argument("--max-camera-index", type=int, default=12)
    parser.add_argument("--camera-width", type=int, default=0)
    parser.add_argument("--camera-height", type=int, default=0)
    parser.add_argument("--camera-fps", type=int, default=0)
    parser.add_argument("--orbbec-serial", default="")
    parser.add_argument("--frame-timeout-ms", type=int, default=30)
    parser.add_argument("--flush-frames", type=int, default=8)
    parser.add_argument("--preview-max-width", type=int, default=960)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.list_cameras:
        print("[camera-list] probing OpenCV cameras")
        for item in probe_opencv_cameras(args.max_camera_index):
            status = "OK" if item["opened"] and item["read_ok"] else "NO"
            print(
                f"  index={item['index']} {status} "
                f"opened={item['opened']} read={item['read_ok']} "
                f"size={item['width']}x{item['height']} fps={item['fps']:.1f}"
            )
        return 0

    if args.output_dir.name.startswith("observe_record_"):
        args.output_dir = args.output_dir.with_name(f"{arm_label(args.arm_side)}_{args.output_dir.name}")
    args.output_dir = args.output_dir.expanduser().resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    snapshot_dir = args.output_dir / "camera_snapshots"
    snapshot_dir.mkdir(parents=True, exist_ok=True)

    proc = None
    saved_count = 0
    try:
        print("[camera] opening preview")
        print("[camera] click the camera window first, then press s/q")
        with Camera(args) as cam:
            proc = start_follower_record(args, args.output_dir)
            cv2.namedWindow("Skewer Topdown Record Camera", cv2.WINDOW_NORMAL)
            while True:
                frame = cam.read()
                if frame is None:
                    key = cv2.waitKey(1) & 0xFF
                else:
                    display = draw_overlay(frame, saved_count, args.output_dir, args.arm_side)
                    max_w = max(240, int(args.preview_max_width))
                    if display.shape[1] > max_w:
                        scale = max_w / float(display.shape[1])
                        display = cv2.resize(
                            display,
                            (max_w, int(round(display.shape[0] * scale))),
                            interpolation=cv2.INTER_AREA,
                        )
                    cv2.imshow("Skewer Topdown Record Camera", display)
                    key = cv2.waitKey(1) & 0xFF

                if proc.poll() is not None:
                    raise RuntimeError(f"follower record exited early with code {proc.returncode}")

                if key in (ord("s"), ord("S")) and frame is not None:
                    path = snapshot_dir / datetime.now().strftime("rgb_%Y%m%d_%H%M%S_%f.png")
                    cv2.imwrite(str(path), frame)
                    saved_count += 1
                    print(f"[camera] saved {path}")
                if key in (ord("q"), 27):
                    break
    finally:
        cv2.destroyAllWindows()
        stop_process(proc)
    print(f"[done] output={args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
