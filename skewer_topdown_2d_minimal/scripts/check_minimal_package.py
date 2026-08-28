#!/usr/bin/env python3
"""Check that skewer_topdown_2d_minimal has the files needed to run.

This does not open cameras and does not move the robot.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VENDOR_ROOT = ROOT / "vendor"

REQUIRED_FILES = [
    ROOT / "README.md",
    ROOT / "requirements.txt",
    ROOT / "config" / "skewer_align_config.json",
    ROOT / "run_skewer_2d_servo.py",
    ROOT / "scripts" / "align_once_move.py",
    ROOT / "scripts" / "auto_calibrate_pixel_to_motion.py",
    ROOT / "scripts" / "pick_target_center.py",
    ROOT / "scripts" / "record_right_observe_with_camera.py",
    ROOT / "outputs" / "right_observe_record_20260723_135046" / "real_follower_right_03.csv",
    ROOT / "outputs" / "right_observe_record_20260723_192233" / "follower_position_snapshots.txt",
]

VENDOR_PATHS = [
    VENDOR_ROOT / "openarm_inplace" / "script" / "run_cpp_real_replay.sh",
    VENDOR_ROOT / "openarm_inplace" / "script" / "run_cpp_real_follower_record.sh",
    VENDOR_ROOT / "openarm_inplace" / "build" / "unilateral_control",
    VENDOR_ROOT / "openarm_inplace" / "build" / "follower_record_control",
    VENDOR_ROOT / "openarm_can" / "include" / "openarm" / "can" / "socket" / "openarm.hpp",
    VENDOR_ROOT / "openarm_description" / "assets" / "robot" / "openarm_v1.0" / "urdf" / "example" / "v1.urdf",
    VENDOR_ROOT / "openarm_vision_grasp_yolo" / "scripts" / "solve_openarm_fk_ik.py",
    VENDOR_ROOT / "openarm_vision_grasp_yolo" / "build" / "kdl_fk_ik_tool",
    VENDOR_ROOT / "template_library" / "rgbd_samurai_minimal" / "01_capture_rgbd.py",
]

REQUIRED_MODULES = ["cv2", "numpy"]
OPTIONAL_MODULES = ["pyorbbecsdk"]


def check_paths(title: str, paths: list[Path]) -> bool:
    ok = True
    print(f"[check] {title}")
    for path in paths:
        exists = path.exists()
        print(f"  {'OK  ' if exists else 'MISS'} {path}")
        ok = ok and exists
    return ok


def check_modules() -> bool:
    ok = True
    print("[check] required python modules")
    for name in REQUIRED_MODULES:
        exists = importlib.util.find_spec(name) is not None
        print(f"  {'OK  ' if exists else 'MISS'} {name}")
        ok = ok and exists
    print("[check] optional python modules")
    for name in OPTIONAL_MODULES:
        exists = importlib.util.find_spec(name) is not None
        print(f"  {'OK  ' if exists else 'MISS'} {name}")
    return ok


def main() -> int:
    required_ok = check_paths("minimal package files", REQUIRED_FILES)
    vendor_ok = check_paths("bundled vendor runtime paths", VENDOR_PATHS)
    modules_ok = check_modules()
    if required_ok and vendor_ok and modules_ok:
        print("[check] OK: package is ready for OpenCV/offline workflows and robot replay integration.")
        return 0
    print("[check] Some required items are missing.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
