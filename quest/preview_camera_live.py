#!/usr/bin/env python3
"""临时：实时预览相机画面并显示实测 fps。

默认 /dev/video8 @ 1280x720 / 60fps / YUYV。
按 q 或 ESC 退出。

用法（在 openarm_quest 环境）:
  conda activate openarm_quest
  python quest/preview_camera_live.py
  python quest/preview_camera_live.py --device 8 --width 1280 --height 720 --fps 60
  python quest/preview_camera_live.py --fourcc MJPG
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque

try:
    import cv2
except ImportError:
    print("需要 opencv-python。请: conda activate openarm_quest", file=sys.stderr)
    sys.exit(1)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="相机实时预览 + fps")
    p.add_argument("--device", default="8")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=float, default=60.0, help="请求帧率（默认 60）")
    p.add_argument("--fourcc", default="YUYV", help="YUYV 或 MJPG")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    dev = args.device
    if isinstance(dev, str) and dev.isdigit():
        dev = int(dev)

    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(dev)
    if not cap.isOpened():
        print(f"无法打开相机: {args.device}", file=sys.stderr)
        return 2

    if args.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc.upper()[:4]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    rf = float(cap.get(cv2.CAP_PROP_FPS))
    print(f"[preview] device={args.device}  {aw}x{ah}  request_fps={args.fps}  reported={rf:.1f}  fourcc={args.fourcc}")
    print("[preview] 按 q / ESC 退出")

    win = f"video{args.device} live"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, min(aw, 1280), min(ah, 720))

    stamps: deque[float] = deque(maxlen=60)
    t_prev = time.perf_counter()
    instant_fps = 0.0

    while True:
        ok, frame = cap.read()
        now = time.perf_counter()
        if not ok or frame is None:
            continue

        dt = now - t_prev
        t_prev = now
        if dt > 1e-6:
            instant_fps = 1.0 / dt
        stamps.append(now)
        avg_fps = 0.0
        if len(stamps) >= 2:
            span = stamps[-1] - stamps[0]
            if span > 1e-6:
                avg_fps = (len(stamps) - 1) / span

        text = f"{aw}x{ah}  avg={avg_fps:5.1f} fps  inst={instant_fps:5.1f}  q=quit"
        cv2.putText(frame, text, (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, text, (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
