#!/usr/bin/env python3
"""Standalone threaded HTTPS MJPEG server for Quest FPV.

Runs in a separate process so quest_bridge WebSocket + teleop CPU load
does not starve camera capture / JPEG encode / long-lived MJPEG sockets.

  python camera_mjpeg_https.py --device 16 --port 8444
"""

from __future__ import annotations

import argparse
import json
import os
import ssl
import sys
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from typing import Optional
from urllib.parse import urlparse

from camera_stream import CameraMjpegSource


class ThreadingHTTPSServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class CamHandler(BaseHTTPRequestHandler):
    camera: Optional[CameraMjpegSource] = None

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path in ("/cam.mjpg", "/camera.mjpg", "/stream.mjpg", "/"):
            self._serve_mjpeg()
            return
        if path == "/cam_status":
            cam = CamHandler.camera
            payload = {"enabled": cam is not None}
            if cam is not None:
                payload.update(cam.status_dict())
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404, "not found")

    def _serve_mjpeg(self) -> None:
        cam = CamHandler.camera
        if cam is None:
            self.send_error(503, "camera not ready")
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        last_sent = -1
        try:
            while True:
                jpeg = cam.latest_jpeg()
                if jpeg is None:
                    if cam.error:
                        break
                    time.sleep(0.03)
                    continue
                # Skip duplicate if client is faster than capture.
                frames = cam.frame_count
                if frames == last_sent:
                    time.sleep(0.01)
                    continue
                last_sent = frames
                header = (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                )
                # One write reduces truncated-JPEG artifacts under load.
                self.wfile.write(header + jpeg + b"\r\n")
                self.wfile.flush()
                time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ssl.SSLError):
            pass


def load_ssl_context(cert_path: str, key_path: str) -> ssl.SSLContext:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(cert_path, key_path)
    return ctx


def main() -> int:
    parser = argparse.ArgumentParser(description="Quest camera MJPEG HTTPS server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=int(os.environ.get("OPENARM_QUEST_CAMERA_PORT", "8444")))
    parser.add_argument("--device", default=os.environ.get("OPENARM_QUEST_CAMERA", "16"))
    parser.add_argument("--width", type=int, default=int(os.environ.get("OPENARM_QUEST_CAMERA_WIDTH", "640")))
    parser.add_argument("--height", type=int, default=int(os.environ.get("OPENARM_QUEST_CAMERA_HEIGHT", "360")))
    parser.add_argument("--fps", type=float, default=float(os.environ.get("OPENARM_QUEST_CAMERA_FPS", "10")))
    parser.add_argument("--jpeg", type=int, default=int(os.environ.get("OPENARM_QUEST_CAMERA_JPEG", "55")))
    parser.add_argument(
        "--preview",
        action="store_true",
        default=os.environ.get("OPENARM_QUEST_CAMERA_PREVIEW", "0").strip().lower()
        in ("1", "true", "yes", "on"),
        help="OpenCV window on this PC (same frames as MJPEG; no second V4L2 open)",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="disable local OpenCV preview window",
    )
    parser.add_argument(
        "--preview-scale",
        type=float,
        default=float(os.environ.get("OPENARM_QUEST_CAMERA_PREVIEW_SCALE", "2.0")),
        help="Upscale factor for local preview window (default 2.0)",
    )
    parser.add_argument(
        "--cert",
        default=os.path.join(os.path.dirname(__file__), "quest_cert.pem"),
    )
    parser.add_argument(
        "--key",
        default=os.path.join(os.path.dirname(__file__), "quest_key.pem"),
    )
    args = parser.parse_args()

    if not os.path.exists(args.cert) or not os.path.exists(args.key):
        print(
            f"[quest-cam-https][ERROR] missing cert/key:\n  {args.cert}\n  {args.key}\n"
            "Start quest_bridge once to generate them, or copy the PEM files here.",
            flush=True,
        )
        return 1

    cam = CameraMjpegSource(
        device=args.device,
        width=args.width,
        height=args.height,
        jpeg_quality=args.jpeg,
        fps=args.fps,
        local_preview=args.preview and not args.no_preview,
        preview_scale=args.preview_scale,
    )
    if not cam.start():
        return 1
    # Wait briefly for first frame.
    for _ in range(50):
        if cam.latest_jpeg() is not None or cam.error:
            break
        time.sleep(0.05)
    if cam.error and cam.latest_jpeg() is None:
        print(f"[quest-cam-https][ERROR] {cam.error}", flush=True)
        return 1

    CamHandler.camera = cam
    server = ThreadingHTTPSServer((args.host, args.port), CamHandler)
    server.socket = load_ssl_context(args.cert, args.key).wrap_socket(
        server.socket, server_side=True
    )
    print(
        f"[quest-cam-https] https://{args.host}:{args.port}/cam.mjpg "
        f"device={args.device}",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
