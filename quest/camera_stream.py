#!/usr/bin/env python3
"""MJPEG camera source for Quest FPV overlay.

Default device: Orbbec Gemini color /dev/video16 (YUYV).
Prefer a native discrete size (640x360) — arbitrary 640x480 can yield
cropped / incomplete-looking frames on some UVC modes.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Optional, Union

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore


DeviceSpec = Union[int, str]


def resolve_device(spec: DeviceSpec) -> DeviceSpec:
    if isinstance(spec, int):
        return spec
    s = str(spec).strip()
    if s.isdigit():
        return int(s)
    if s.startswith("/dev/video"):
        return s
    if s.startswith("video") and s[5:].isdigit():
        return int(s[5:])
    return s


class CameraMjpegSource:
    def __init__(
        self,
        device: DeviceSpec = 16,
        width: int = 640,
        height: int = 360,
        jpeg_quality: int = 55,
        fps: float = 10.0,
        local_preview: bool = False,
        preview_scale: float = 2.0,
    ) -> None:
        self.device = resolve_device(device)
        self.width = max(160, int(width))
        self.height = max(120, int(height))
        self.jpeg_quality = int(max(30, min(95, jpeg_quality)))
        self.period = 1.0 / max(1.0, float(fps))
        self.local_preview = bool(local_preview)
        self.preview_scale = max(1.0, float(preview_scale))
        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._error: Optional[str] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frames = 0
        self._last_ok = 0.0
        self._frame_wh = (0, 0)
        self._preview_warned = False

    @property
    def error(self) -> Optional[str]:
        return self._error

    @property
    def frame_count(self) -> int:
        return self._frames

    @property
    def frame_size(self) -> tuple[int, int]:
        return self._frame_wh

    def latest_jpeg(self) -> Optional[bytes]:
        with self._lock:
            # Return a reference to immutable bytes (safe across threads).
            return self._jpeg

    def status_dict(self) -> dict:
        with self._lock:
            return {
                "error": self._error,
                "frames": self._frames,
                "has_jpeg": self._jpeg is not None,
                "width": self._frame_wh[0],
                "height": self._frame_wh[1],
                "device": str(self.device),
            }

    def start(self) -> bool:
        if cv2 is None:
            self._error = "opencv-python not installed (conda activate openarm_quest)"
            print(f"[quest-cam][ERROR] {self._error}", flush=True)
            return False
        if self._running:
            return True
        try:
            cv2.setNumThreads(1)
        except Exception:
            pass
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="quest-cam", daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
        if cv2 is not None and self.local_preview:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

    def _maybe_show_preview(self, frame) -> None:
        if not self.local_preview or cv2 is None:
            return
        if not os.environ.get("DISPLAY"):
            if not self._preview_warned:
                self._preview_warned = True
                print(
                    "[quest-cam][WARN] local preview requested but DISPLAY is unset",
                    flush=True,
                )
            return
        show = frame
        if self.preview_scale > 1.01:
            h, w = frame.shape[:2]
            show = cv2.resize(
                frame,
                (int(w * self.preview_scale), int(h * self.preview_scale)),
                interpolation=cv2.INTER_LINEAR,
            )
        cv2.imshow("video16 (quest cam)", show)
        cv2.waitKey(1)

    def _open_capture(self):
        assert cv2 is not None
        for backend in (cv2.CAP_V4L2, cv2.CAP_ANY):
            cap = cv2.VideoCapture(self.device, backend)
            if cap.isOpened():
                return cap
            cap.release()
        return None

    def _configure(self, cap) -> None:
        assert cv2 is not None
        # Orbbec color node is YUYV; setting FOURCC avoids odd partial formats.
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        except Exception:
            pass
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        try:
            cap.set(cv2.CAP_PROP_FPS, max(1.0, 1.0 / self.period))
        except Exception:
            pass
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        # Warm-up / flush driver queue.
        for _ in range(8):
            cap.grab()

    def _loop(self) -> None:
        assert cv2 is not None
        label = self.device if isinstance(self.device, str) else f"/dev/video{self.device}"
        print(
            f"[quest-cam] opening {label} (request {self.width}x{self.height} "
            f"@ ~{1.0 / self.period:.0f} Hz q={self.jpeg_quality})",
            flush=True,
        )
        cap = self._open_capture()
        if cap is None or not cap.isOpened():
            self._error = (
                f"cannot open {label} — is another app (ffplay/Cheese/bridge) using it?"
            )
            print(f"[quest-cam][ERROR] {self._error}", flush=True)
            self._running = False
            return

        self._configure(cap)
        ok, frame = cap.read()
        if not ok or frame is None:
            self._error = f"{label} opened but first read failed"
            print(f"[quest-cam][ERROR] {self._error}", flush=True)
            cap.release()
            self._running = False
            return

        h0, w0 = frame.shape[:2]
        self._frame_wh = (w0, h0)
        print(f"[quest-cam] streaming {label} actual={w0}x{h0}", flush=True)
        if self.local_preview and os.environ.get("DISPLAY"):
            print(
                f"[quest-cam] local preview ON (scale={self.preview_scale:.1f}x, "
                "same capture as MJPEG — close window or stop bridge to exit)",
                flush=True,
            )
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]

        while self._running:
            t0 = time.monotonic()
            # Drop stale buffered frames so teleop CPU spikes don't show lagged halves.
            if not cap.grab():
                time.sleep(0.02)
                continue
            ok, frame = cap.retrieve()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            h, w = frame.shape[:2]
            self._frame_wh = (w, h)
            # Only resize if width differs a lot; keep aspect (avoid vertical crop look).
            if abs(w - self.width) > 8:
                new_h = max(1, int(round(h * (self.width / float(w)))))
                frame = cv2.resize(frame, (self.width, new_h), interpolation=cv2.INTER_AREA)
                self._frame_wh = (self.width, new_h)
            ok_enc, buf = cv2.imencode(".jpg", frame, encode_param)
            if not ok_enc:
                continue
            jpeg = buf.tobytes()
            with self._lock:
                self._jpeg = jpeg
                self._frames += 1
                self._last_ok = time.monotonic()
                self._error = None
            self._maybe_show_preview(frame)
            elapsed = time.monotonic() - t0
            sleep_s = self.period - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

        cap.release()
        print("[quest-cam] stopped", flush=True)


def camera_from_env() -> Optional[CameraMjpegSource]:
    enabled = os.environ.get("OPENARM_QUEST_CAMERA_ENABLE", "1").strip().lower()
    if enabled in ("0", "false", "off", "no"):
        return None
    device = os.environ.get("OPENARM_QUEST_CAMERA", "16")
    width = int(os.environ.get("OPENARM_QUEST_CAMERA_WIDTH", "640"))
    height = int(os.environ.get("OPENARM_QUEST_CAMERA_HEIGHT", "360"))
    quality = int(os.environ.get("OPENARM_QUEST_CAMERA_JPEG", "55"))
    fps = float(os.environ.get("OPENARM_QUEST_CAMERA_FPS", "10"))
    preview = os.environ.get("OPENARM_QUEST_CAMERA_PREVIEW", "0").strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    preview_scale = float(os.environ.get("OPENARM_QUEST_CAMERA_PREVIEW_SCALE", "2.0"))
    return CameraMjpegSource(
        device, width, height, quality, fps, local_preview=preview, preview_scale=preview_scale
    )
