#!/usr/bin/env python3
"""Click a fixed gripper-center pixel and print the command-line argument."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Click target center pixel on an image.")
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--max-width", type=int, default=960)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    image_path = args.image.expanduser().resolve()
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError(f"Failed to read image: {image_path}")

    h, w = image.shape[:2]
    scale = 1.0
    display = image
    if w > int(args.max_width):
        scale = int(args.max_width) / float(w)
        display = cv2.resize(image, (int(round(w * scale)), int(round(h * scale))), interpolation=cv2.INTER_AREA)

    clicked: list[tuple[float, float]] = []

    def on_mouse(event, x, y, _flags, _userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        u = float(x) / scale
        v = float(y) / scale
        clicked[:] = [(u, v)]
        shown = display.copy()
        cv2.drawMarker(shown, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 28, 2)
        cv2.putText(
            shown,
            f"target-center=({u:.1f},{v:.1f})",
            (18, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            lineType=cv2.LINE_AA,
        )
        cv2.imshow("Pick Gripper Center", shown)
        print(f"--target-center-px={u:.1f},{v:.1f}", flush=True)

    print("[pick] 鼠标左键点夹爪中心；q/Esc 退出。")
    print("[pick] 建议点两个夹爪内侧尖端的中间位置，也就是希望烧烤签黑色抓取中心最终到达的位置。")
    cv2.namedWindow("Pick Gripper Center", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Pick Gripper Center", on_mouse)
    cv2.imshow("Pick Gripper Center", display)
    while True:
        key = cv2.waitKey(20) & 0xFF
        if key in (ord("q"), 27):
            break
    cv2.destroyAllWindows()
    if clicked:
        u, v = clicked[-1]
        print(f"[pick] final: --target-center-px={u:.1f},{v:.1f}")
    else:
        print("[pick] no point clicked")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
