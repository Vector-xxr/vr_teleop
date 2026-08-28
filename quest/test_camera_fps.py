#!/usr/bin/env python3
"""临时相机帧率测试脚本。

测量当前相机能否达到目标帧率（默认 30 fps）。
默认设备：彩色输出 /dev/video8。

用法:
  python3 quest/test_camera_fps.py                 # 默认 video8, 640x480, 30fps, 测 5s
  python3 quest/test_camera_fps.py --device 8
  python3 quest/test_camera_fps.py --device /dev/video8 --width 1280 --height 720 --fps 30
  python3 quest/test_camera_fps.py --fourcc MJPG    # 让相机端用 MJPG 传输
  python3 quest/test_camera_fps.py --seconds 10

说明:
  - 报告「相机上报 fps」（V4L2 声明值）与「实测捕获 fps」（真实 grab 速率）。
  - 只读相机、不推流、不开预览窗口，测完即退出。
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Union

try:
    import cv2
except ImportError:
    print("[fps-test] 需要 opencv-python: pip install opencv-python", file=sys.stderr)
    sys.exit(1)


DeviceSpec = Union[int, str]


def resolve_device(spec: str) -> DeviceSpec:
    s = str(spec).strip()
    if s.isdigit():
        return int(s)
    if s.startswith("/dev/video"):
        return s
    if s.startswith("video") and s[5:].isdigit():
        return int(s[5:])
    return s


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="相机帧率测试")
    p.add_argument("--device", default="8", help="设备：8 / video8 / /dev/video8（默认 8）")
    p.add_argument("--width", type=int, default=640, help="请求宽度（默认 640）")
    p.add_argument("--height", type=int, default=480, help="请求高度（默认 480）")
    p.add_argument("--fps", type=float, default=30.0, help="目标帧率（默认 30）")
    p.add_argument("--seconds", type=float, default=5.0, help="测量时长秒（默认 5）")
    p.add_argument("--fourcc", default="", help="可选像素格式，如 MJPG / YUYV（默认不设，用相机默认）")
    p.add_argument("--warmup", type=int, default=10, help="预热丢弃帧数（默认 10）")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    dev = resolve_device(args.device)

    # Linux 上优先用 V4L2 后端，避免 GStreamer 默认路径带来的额外开销。
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(dev)
    if not cap.isOpened():
        print(f"[fps-test] 无法打开相机: {dev}", file=sys.stderr)
        return 2

    if args.fourcc:
        fourcc = cv2.VideoWriter_fourcc(*args.fourcc.upper()[:4])
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    act_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    act_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    reported_fps = cap.get(cv2.CAP_PROP_FPS)
    raw_cc = int(cap.get(cv2.CAP_PROP_FOURCC))
    cc = "".join([chr((raw_cc >> (8 * i)) & 0xFF) for i in range(4)]).strip("\x00 ") or "?"

    print(f"[fps-test] 设备={dev}  分辨率={act_w}x{act_h}  格式={cc}")
    print(f"[fps-test] 相机上报 fps={reported_fps:.2f}  目标 fps={args.fps:.1f}  时长={args.seconds:.1f}s")

    for _ in range(max(0, args.warmup)):
        cap.read()

    frames = 0
    dropped = 0
    min_dt = float("inf")
    max_dt = 0.0
    t_start = time.perf_counter()
    t_prev = t_start
    deadline = t_start + max(0.5, args.seconds)

    while time.perf_counter() < deadline:
        ok, frame = cap.read()
        now = time.perf_counter()
        if not ok or frame is None:
            dropped += 1
            continue
        frames += 1
        dt = now - t_prev
        t_prev = now
        if frames > 1:
            min_dt = min(min_dt, dt)
            max_dt = max(max_dt, dt)

    elapsed = time.perf_counter() - t_start
    cap.release()

    measured = frames / elapsed if elapsed > 0 else 0.0
    print("[fps-test] ---- 结果 ----")
    print(f"[fps-test] 捕获帧数={frames}  丢帧/失败={dropped}  用时={elapsed:.2f}s")
    print(f"[fps-test] 实测 fps={measured:.2f}")
    if frames > 1 and min_dt != float("inf"):
        print(f"[fps-test] 帧间隔 min={min_dt * 1000:.1f}ms  max={max_dt * 1000:.1f}ms  "
              f"(瞬时 max fps≈{1.0 / min_dt:.1f})")

    target = args.fps
    if measured >= target * 0.95:
        print(f"[fps-test] 结论: 达标 ✓ 可稳定到 ~{target:.0f}fps")
        return 0
    if measured >= target * 0.8:
        print(f"[fps-test] 结论: 接近但未稳定达标（实测 {measured:.1f} / 目标 {target:.0f}）")
        return 1
    print(f"[fps-test] 结论: 未达标 ✗ 实测 {measured:.1f}fps 明显低于目标 {target:.0f}fps")
    print("[fps-test] 提示: 试 --fourcc MJPG、降分辨率，或确认这是彩色流而非深度流。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
