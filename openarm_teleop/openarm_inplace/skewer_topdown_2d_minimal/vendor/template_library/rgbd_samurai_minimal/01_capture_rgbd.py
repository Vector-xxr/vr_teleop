#!/usr/bin/env python3
"""
功能:
  使用 Orbbec RGB-D 相机采集对齐到 RGB 的彩色图、深度图和相机内参。

常用运行:
  /home/wenjiajie/anaconda3/envs/anygrasp/bin/python \
    rgbd_samurai_minimal/01_capture_rgbd.py \
    --output captured/cola_can_raw

输出:
  --output/
    color_000.png          RGB/彩色图, OpenCV 写入
    depth_000.png          SDK 对齐到 color frame 的 uint16 原始深度
    intrinsics.json        当前 RGB-D 分辨率下的相机内参和 depth_scale

按键:
  s / Space  保存当前 RGB 和深度
  q / Esc    退出

参数:
  -o, --output             输出目录, 默认 captures/manual_rgbd
  --depth-preview-max      深度预览最大距离, 默认 2.5 m

注意:
  depth_*.png 不做中值滤波、融合、修复或 resize。
  输出深度已由 SDK 对齐到彩色流, depth[y, x] 对应 color[y, x]。
"""

import argparse
import json
import os
import time
from pathlib import Path

# OpenCV GUI fix for GNOME/Wayland. Must be set before importing cv2.
if "wayland" in os.environ.get("XDG_SESSION_TYPE", "").lower():
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import cv2
import numpy as np

try:
    from pyorbbecsdk import (
        AlignFilter,
        Config,
        OBAlignMode,
        OBFormat,
        OBSensorType,
        OBStreamType,
        Pipeline,
    )
except ImportError as exc:
    raise SystemExit(
        "pyorbbecsdk is not installed. Run this in the anygrasp environment."
    ) from exc


def profile_intrinsics(profile):
    intr = profile.get_intrinsic()
    data = {
        "width": int(intr.width),
        "height": int(intr.height),
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "cx": float(intr.cx),
        "cy": float(intr.cy),
    }
    if hasattr(profile, "get_distortion"):
        try:
            dist = profile.get_distortion()
            k1 = float(getattr(dist, "k1", 0.0))
            k2 = float(getattr(dist, "k2", 0.0))
            k3 = float(getattr(dist, "k3", 0.0))
            k4 = float(getattr(dist, "k4", 0.0))
            k5 = float(getattr(dist, "k5", 0.0))
            k6 = float(getattr(dist, "k6", 0.0))
            p1 = float(getattr(dist, "p1", 0.0))
            p2 = float(getattr(dist, "p2", 0.0))
            # OpenCV order: k1, k2, p1, p2, k3[, k4, k5, k6].
            data["dist_coeffs"] = [k1, k2, p1, p2, k3]
            data["dist_coeffs_8"] = [k1, k2, p1, p2, k3, k4, k5, k6]
        except Exception:
            pass
    return data


def write_intrinsics(path, intrinsics, image_shape, depth_scale_m_per_unit):
    height, width = image_shape[:2]
    depth_scale = 1.0 / depth_scale_m_per_unit
    data = {
        "width": int(width),
        "height": int(height),
        "fx": float(intrinsics["fx"]),
        "fy": float(intrinsics["fy"]),
        "cx": float(intrinsics["cx"]),
        "cy": float(intrinsics["cy"]),
        "camera_matrix": [
            [float(intrinsics["fx"]), 0.0, float(intrinsics["cx"])],
            [0.0, float(intrinsics["fy"]), float(intrinsics["cy"])],
            [0.0, 0.0, 1.0],
        ],
        "depth_scale": float(round(depth_scale, 6)),
    }
    if "dist_coeffs" in intrinsics:
        data["dist_coeffs"] = [float(x) for x in intrinsics["dist_coeffs"]]
    if "dist_coeffs_8" in intrinsics:
        data["dist_coeffs_8"] = [float(x) for x in intrinsics["dist_coeffs_8"]]
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def sdk_depth_scale_to_meters(scale):
    """Normalize SDK scale to meters per uint16 unit."""
    scale = float(scale)
    if scale <= 0:
        return 0.001
    # Some Orbbec SDK builds report millimeters/unit (usually 1.0), while
    # others report meters/unit (usually 0.001).
    return scale / 1000.0 if scale > 0.1 else scale


def frame_to_bgr(frame):
    width = frame.get_width()
    height = frame.get_height()
    fmt = frame.get_format()
    data = np.frombuffer(frame.get_data(), dtype=np.uint8)

    if fmt == OBFormat.RGB:
        return cv2.cvtColor(data.reshape((height, width, 3)), cv2.COLOR_RGB2BGR)
    if fmt == OBFormat.BGR:
        return data.reshape((height, width, 3)).copy()
    if hasattr(OBFormat, "BGRA") and fmt == OBFormat.BGRA:
        return cv2.cvtColor(data.reshape((height, width, 4)), cv2.COLOR_BGRA2BGR)
    if hasattr(OBFormat, "YUYV") and fmt == OBFormat.YUYV:
        return cv2.cvtColor(data.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_YUYV)
    if hasattr(OBFormat, "UYVY") and fmt == OBFormat.UYVY:
        return cv2.cvtColor(data.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_UYVY)
    if fmt == OBFormat.MJPG:
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("Failed to decode MJPG color frame")
        return image

    raise RuntimeError(f"Unsupported color format: {fmt}")


def depth_frame_to_u16(frame):
    width = frame.get_width()
    height = frame.get_height()
    return np.frombuffer(frame.get_data(), dtype=np.uint16).reshape((height, width)).copy()


def make_depth_preview(depth_u16, depth_scale, max_m=2.5):
    depth_m = depth_u16.astype(np.float32) * depth_scale
    valid = depth_m > 0
    norm = np.zeros(depth_m.shape, dtype=np.uint8)
    norm[valid] = np.clip(depth_m[valid] / max_m * 255, 0, 255).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_JET)


class OrbbecRgbdCamera:
    def __init__(self, serial=None):
        self.pipeline = None
        self.align_filter = None
        self.intrinsics = None
        self.depth_scale = 0.001
        self.serial = (serial or "").strip() or None
        self.active_serial = ""

    def start(self):
        from pyorbbecsdk import Context  # local import keeps module load light

        pipeline = None
        if self.serial:
            ctx = Context()
            devices = ctx.query_devices()
            device = devices.get_device_by_serial_number(str(self.serial))
            if device is None:
                raise RuntimeError(f"Orbbec device not found for serial={self.serial}")
            try:
                pipeline = Pipeline(device)
            except TypeError as exc:
                raise RuntimeError(
                    "This pyorbbecsdk build did not accept Pipeline(device). "
                    "Upgrade SDK or unplug the other camera."
                ) from exc
            try:
                self.active_serial = device.get_device_info().get_serial_number()
            except Exception:
                self.active_serial = str(self.serial)
        else:
            pipeline = Pipeline()

        config = Config()

        depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        try:
            depth_profile = depth_profiles.get_video_stream_profile(
                640, 480, OBFormat.Y16, 30
            )
            config.enable_stream(depth_profile)
        except Exception:
            config.enable_video_stream(OBSensorType.DEPTH_SENSOR, 640, 480, 30, OBFormat.Y16)

        color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = color_profiles.get_default_video_stream_profile()
        config.enable_stream(color_profile)

        if hasattr(config, "set_align_mode"):
            try:
                config.set_align_mode(OBAlignMode.SW_MODE)
            except Exception:
                pass

        pipeline.start(config)
        try:
            pipeline.enable_frame_sync()
        except Exception:
            pass

        self.intrinsics = profile_intrinsics(color_profile)
        self.pipeline = pipeline
        self.align_filter = AlignFilter(OBStreamType.COLOR_STREAM)

        serial_note = f" serial={self.active_serial or self.serial or 'default'}"
        print(
            f"[Camera] started. color={self.intrinsics['width']}x{self.intrinsics['height']} "
            f"align_to_color=True{serial_note}"
        )

    def read(self, timeout_ms=50):
        frames = self.pipeline.wait_for_frames(timeout_ms)
        if frames is None:
            return None

        frames = self.align_filter.process(frames)
        if frames is None:
            return None
        frames = frames.as_frame_set()

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if color_frame is None or depth_frame is None:
            return None

        color = frame_to_bgr(color_frame)
        depth = depth_frame_to_u16(depth_frame)

        if hasattr(depth_frame, "get_depth_scale"):
            self.depth_scale = sdk_depth_scale_to_meters(depth_frame.get_depth_scale())

        if color.shape[:2] != depth.shape[:2]:
            raise RuntimeError(
                f"Aligned frame size mismatch: color={color.shape[1]}x{color.shape[0]}, "
                f"depth={depth.shape[1]}x{depth.shape[0]}"
            )

        return color, depth

    def stop(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
            print("[Camera] stopped")


def next_index(output_dir):
    nums = []
    for path in output_dir.glob("color_*.png"):
        try:
            nums.append(int(path.stem.split("_")[-1]))
        except ValueError:
            pass
    return max(nums, default=-1) + 1


def save_frame(output_dir, idx, color, depth, intrinsics, depth_scale):
    color_path = output_dir / f"color_{idx:03d}.png"
    depth_path = output_dir / f"depth_{idx:03d}.png"
    cv2.imwrite(str(color_path), color)
    cv2.imwrite(str(depth_path), depth)

    write_intrinsics(
        output_dir / "intrinsics.json",
        intrinsics,
        color.shape,
        depth_scale,
    )

    meta = {
        "index": idx,
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "color": color_path.name,
        "depth": depth_path.name,
        "depth_dtype": "uint16",
        "depth_scale": float(round(1.0 / depth_scale, 6)),
        "aligned_depth_to_color": True,
        "postprocess": "none",
    }
    with open(output_dir / f"frame_{idx:03d}.json", "w") as f:
        json.dump(meta, f, indent=2)

    print(f"[Saved] {color_path.name}, {depth_path.name}")


def main():
    parser = argparse.ArgumentParser(description="Capture Orbbec RGB-D frames with keypress")
    parser.add_argument("-o", "--output", default="captures/manual_rgbd")
    parser.add_argument("--depth-preview-max", type=float, default=2.5)
    parser.add_argument(
        "--frame-timeout-ms",
        type=int,
        default=50,
        help="Camera wait timeout per loop. Lower values make keyboard response faster.",
    )
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    cam = OrbbecRgbdCamera()
    cam.start()

    idx = next_index(output_dir)
    cv2.namedWindow("Orbbec RGBD", cv2.WINDOW_NORMAL)
    print(f"[Output] {output_dir}")
    print("[Keys] s/Space: save, q/Esc: quit")

    last_color = None
    last_depth = None
    last_depth_scale = cam.depth_scale

    try:
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (ord("s"), ord("S"), ord(" ")) and last_color is not None:
                save_frame(
                    output_dir,
                    idx,
                    last_color,
                    last_depth,
                    cam.intrinsics,
                    last_depth_scale,
                )
                idx += 1

            packet = cam.read(args.frame_timeout_ms)
            if packet is None:
                continue

            color, depth = packet
            last_color = color
            last_depth = depth
            last_depth_scale = cam.depth_scale

            depth_vis = make_depth_preview(depth, cam.depth_scale, args.depth_preview_max)
            if depth_vis.shape[:2] != color.shape[:2]:
                depth_vis = cv2.resize(depth_vis, (color.shape[1], color.shape[0]), interpolation=cv2.INTER_NEAREST)

            vis = np.hstack([color, depth_vis])
            cv2.putText(
                vis,
                f"s: save  q: quit  next={idx:03d}  depth_scale={cam.depth_scale:g}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.imshow("Orbbec RGBD", vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (ord("s"), ord("S"), ord(" ")):
                save_frame(output_dir, idx, color, depth, cam.intrinsics, cam.depth_scale)
                idx += 1
    finally:
        cv2.destroyAllWindows()
        cam.stop()


if __name__ == "__main__":
    main()
