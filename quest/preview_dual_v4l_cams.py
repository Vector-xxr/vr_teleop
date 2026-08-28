#!/usr/bin/env python3
"""Open one preview window per V4L camera (Quest teleop recording).

Prefers ffplay. Discovers capture nodes if preferred devices are missing.
If a device is busy (e.g. quest camera bridge on video16), falls back to
the local MJPEG HTTPS stream when available.

Env:
  OPENARM_QUEST_RECORD_CAMS=8,16   preferred device list
  OPENARM_QUEST_CAM_PREVIEW_W/H/FPS
  OPENARM_QUEST_CAM_PREVIEW_BACKEND=auto|ffplay|opencv
  OPENARM_QUEST_CAMERA_PORT=8444   MJPEG fallback port (busy video16)
  OPENARM_QUEST_CAMERA_HOST=127.0.0.1
"""

from __future__ import annotations

import argparse
from pathlib import Path
import os
import re
import shutil
import signal
import subprocess
import sys
import time


def parse_devices(s: str) -> list[str]:
    out: list[str] = []
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        if part.isdigit():
            out.append(f"/dev/video{part}")
        elif part.startswith("video") and part[5:].isdigit():
            out.append(f"/dev/{part}")
        else:
            out.append(part)
    return out


def device_busy(path: str) -> bool:
    fuser = shutil.which("fuser")
    if not fuser or not os.path.exists(path):
        return False
    try:
        r = subprocess.run(
            [fuser, path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=1.5,
            check=False,
        )
        return r.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def _is_capture_node(path: str) -> bool:
    """True if v4l2 reports Video Capture (skip metadata-only nodes)."""
    v4l2 = shutil.which("v4l2-ctl")
    if not v4l2:
        return os.path.exists(path)
    try:
        out = subprocess.check_output(
            [v4l2, "-d", path, "--all"],
            stderr=subprocess.STDOUT,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return os.path.exists(path)
    caps = ""
    m = re.search(r"Device Caps\s*:\s*(.*?)(?:\n\S|\Z)", out, re.S)
    if m:
        caps = m.group(1)
    if "Video Capture" in caps:
        return True
    return "Video Capture" in out and "Video Output" not in caps[:80]


def discover_capture_devices(limit: int = 2) -> list[str]:
    found: list[str] = []
    for i in range(64):
        path = f"/dev/video{i}"
        if not os.path.exists(path):
            continue
        if _is_capture_node(path):
            found.append(path)
        if len(found) >= limit:
            break
    return found


def resolve_devices(preferred: list[str]) -> list[str]:
    ok = [d for d in preferred if os.path.exists(d)]
    for d in preferred:
        if d not in ok:
            print(f"[cam-preview] missing preferred {d}", file=sys.stderr, flush=True)
    usable = []
    for d in ok:
        if _is_capture_node(d):
            usable.append(d)
        else:
            print(f"[cam-preview] skip non-capture node {d}", file=sys.stderr, flush=True)
    if len(usable) >= 2:
        return usable[:2]
    if usable:
        extra = [d for d in discover_capture_devices(4) if d not in usable]
        return (usable + extra)[:2]
    guessed = discover_capture_devices(2)
    if guessed:
        print(f"[cam-preview] using discovered: {', '.join(guessed)}", file=sys.stderr, flush=True)
        return guessed
    # Common failure: drivers present in sysfs but /dev/video* nodes missing.
    sysfs = sorted(Path("/sys/class/video4linux").glob("video*")) if Path("/sys/class/video4linux").is_dir() else []
    if sysfs and not any(Path(f"/dev/{x.name}").exists() for x in sysfs):
        names = ", ".join(x.name for x in sysfs[:8])
        print(
            f"[cam-preview] V4L drivers loaded ({names}…) but /dev/video* nodes are MISSING. "
            "Run: sudo ./quest/fix_v4l_dev_nodes.sh   (or: sudo udevadm trigger -s video4linux -c add)",
            file=sys.stderr,
            flush=True,
        )
    else:
        print(
            "[cam-preview] no /dev/video* capture devices found. "
            "Plug cameras in / load Orbbec drivers, or set OPENARM_QUEST_RECORD_CAMS.",
            file=sys.stderr,
            flush=True,
        )
    return []


def gui_env() -> dict[str, str]:
    env = os.environ.copy()
    if not env.get("DISPLAY"):
        env["DISPLAY"] = ":0"
    candidates: list[str | Path] = []
    if env.get("XAUTHORITY"):
        candidates.append(env["XAUTHORITY"])
    for uid in (os.geteuid(), 1000):
        run = Path(f"/run/user/{uid}")
        if run.is_dir():
            candidates.extend(sorted(run.glob(".mutter-Xwaylandauth.*")))
            candidates.append(run / "gdm" / "Xauthority")
    candidates.append(Path.home() / ".Xauthority")
    candidates.append(Path("/home/lan/.Xauthority"))
    for xa in candidates:
        p = Path(xa)
        if p.is_file():
            env["XAUTHORITY"] = str(p)
            break
    if "SDL_VIDEODRIVER" not in env:
        env["SDL_VIDEODRIVER"] = "x11"
    env.setdefault("SDL_AUDIODRIVER", "dummy")
    return env


def mjpeg_url_for_device(dev: str) -> str | None:
    """If camera bridge holds this device, return its MJPEG URL."""
    # Default Quest FPV cam is video16 on OPENARM_QUEST_CAMERA_PORT.
    cam = os.environ.get("OPENARM_QUEST_CAMERA", "16").strip()
    if not cam.isdigit():
        return None
    if os.path.basename(dev) != f"video{cam}":
        return None
    host = os.environ.get("OPENARM_QUEST_CAMERA_HOST", "127.0.0.1")
    port = os.environ.get("OPENARM_QUEST_CAMERA_PORT", "8444")
    return f"https://{host}:{port}/cam.mjpg"


def _ffplay_base() -> list[str]:
    return [
        "-hide_banner",
        "-loglevel",
        "warning",
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-framedrop",
        "-alwaysontop",
    ]


def spawn_ffplay_v4l(dev: str, title: str, width: int, height: int, env: dict[str, str]) -> subprocess.Popen | None:
    ffplay = shutil.which("ffplay")
    if not ffplay:
        return None
    # Orbbec / many UVC cams prefer MJPEG; fall back to plain v4l2 if needed.
    attempts = [
        [
            ffplay,
            *_ffplay_base(),
            "-window_title",
            title,
            "-f",
            "v4l2",
            "-input_format",
            "mjpeg",
            "-video_size",
            f"{width}x{height}",
            "-i",
            dev,
        ],
        [
            ffplay,
            *_ffplay_base(),
            "-window_title",
            title,
            "-f",
            "v4l2",
            "-video_size",
            f"{width}x{height}",
            "-i",
            dev,
        ],
        [
            ffplay,
            *_ffplay_base(),
            "-window_title",
            title,
            "-f",
            "v4l2",
            "-i",
            dev,
        ],
    ]
    last_err = ""
    for cmd in attempts:
        try:
            p = subprocess.Popen(
                cmd,
                env=env,
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )
        except OSError as exc:
            last_err = str(exc)
            continue
        time.sleep(0.45)
        rc = p.poll()
        if rc is None:
            print(f"[cam-preview] ffplay {dev} -> '{title}' pid={p.pid}", flush=True)
            return p
        try:
            last_err = (p.stderr.read() or "").strip()
        except Exception:
            last_err = f"rc={rc}"
        print(f"[cam-preview] attempt failed for {dev}: {last_err[:200]}", file=sys.stderr, flush=True)
    print(f"[cam-preview] ffplay could not open {dev}: {last_err[:300]}", file=sys.stderr, flush=True)
    return None


def spawn_ffplay_url(url: str, title: str, env: dict[str, str]) -> subprocess.Popen | None:
    ffplay = shutil.which("ffplay")
    if not ffplay:
        return None
    cmd = [
        ffplay,
        *_ffplay_base(),
        "-tls_verify",
        "0",
        "-window_title",
        title,
        url,
    ]
    try:
        p = subprocess.Popen(
            cmd,
            env=env,
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
    except OSError as exc:
        print(f"[cam-preview] ffplay URL failed: {exc}", file=sys.stderr, flush=True)
        return None
    time.sleep(0.5)
    if p.poll() is not None:
        err = ""
        try:
            err = (p.stderr.read() or "").strip()
        except Exception:
            pass
        print(f"[cam-preview] MJPEG preview exited: {err[:300]}", file=sys.stderr, flush=True)
        return None
    print(f"[cam-preview] ffplay {url} -> '{title}' pid={p.pid}", flush=True)
    return p


def run_ffplay(devices: list[str], width: int, height: int) -> int:
    if not shutil.which("ffplay"):
        print("[cam-preview] ffplay not found", file=sys.stderr, flush=True)
        return 2
    env = gui_env()
    print(f"[cam-preview] DISPLAY={env.get('DISPLAY')} XAUTHORITY={env.get('XAUTHORITY')}", flush=True)
    procs: list[subprocess.Popen] = []
    stop = False

    def _stop(*_a):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    for i, dev in enumerate(devices):
        title = f"cam{i} {os.path.basename(dev)}"
        if device_busy(dev):
            url = mjpeg_url_for_device(dev)
            print(
                f"[cam-preview] {dev} busy"
                + (f" → MJPEG {url}" if url else " (no MJPEG fallback)"),
                file=sys.stderr,
                flush=True,
            )
            if url:
                p = spawn_ffplay_url(url, f"{title} (MJPEG)", env)
                if p is not None:
                    procs.append(p)
                    continue
            print(
                f"[cam-preview] skip busy {dev}: stop camera bridge or use preview_cam_mjpeg.sh",
                file=sys.stderr,
                flush=True,
            )
            continue
        p = spawn_ffplay_v4l(dev, title, width, height, env)
        if p is not None:
            procs.append(p)

    if not procs:
        # Last chance: if preferred FPV cam missing/busy, still try MJPEG alone.
        url = mjpeg_url_for_device(f"/dev/video{os.environ.get('OPENARM_QUEST_CAMERA', '16')}")
        if url:
            print(f"[cam-preview] trying lone MJPEG fallback {url}", flush=True)
            p = spawn_ffplay_url(url, "cam FPV (MJPEG)", env)
            if p is not None:
                procs.append(p)
        if not procs:
            return 1

    while not stop:
        still = [p for p in procs if p.poll() is None]
        if not still:
            break
        time.sleep(0.25)

    for p in procs:
        if p.poll() is None:
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except (OSError, ProcessLookupError):
                p.terminate()
    for p in procs:
        try:
            p.wait(timeout=1.5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except (OSError, ProcessLookupError):
                p.kill()
    print("[cam-preview] closed", flush=True)
    return 0


def run_opencv(devices: list[str], width: int, height: int, fps: int) -> int:
    try:
        import cv2
    except ImportError:
        print("[cam-preview] opencv not installed", file=sys.stderr, flush=True)
        return 2

    stop = False

    def _stop(*_a):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    caps = []
    titles = []
    for i, dev in enumerate(devices):
        if device_busy(dev):
            print(f"[cam-preview] opencv skip busy {dev}", file=sys.stderr, flush=True)
            continue
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(dev)
        if not cap.isOpened():
            print(f"[cam-preview] FAILED open {dev}", file=sys.stderr, flush=True)
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        title = f"cam{i} {os.path.basename(dev)}"
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        caps.append(cap)
        titles.append(title)
        print(f"[cam-preview] opencv {dev} -> '{title}'", flush=True)

    if not caps:
        return 1

    period = 1.0 / max(1, fps)
    while not stop:
        for cap, title in zip(caps, titles):
            ok, frame = cap.read()
            if ok and frame is not None:
                cv2.imshow(title, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        alive = False
        for title in titles:
            try:
                if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) >= 1:
                    alive = True
            except cv2.error:
                pass
        if not alive:
            break
        time.sleep(period * 0.2)

    for cap in caps:
        cap.release()
    cv2.destroyAllWindows()
    print("[cam-preview] closed", flush=True)
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description="Dual V4L camera preview")
    p.add_argument(
        "--devices",
        default=os.environ.get("OPENARM_QUEST_RECORD_CAMS", "8,16"),
        help="Comma-separated preferred devices",
    )
    p.add_argument("--width", type=int, default=int(os.environ.get("OPENARM_QUEST_CAM_PREVIEW_W", "640")))
    p.add_argument("--height", type=int, default=int(os.environ.get("OPENARM_QUEST_CAM_PREVIEW_H", "480")))
    p.add_argument("--fps", type=int, default=int(os.environ.get("OPENARM_QUEST_CAM_PREVIEW_FPS", "15")))
    p.add_argument(
        "--backend",
        choices=("auto", "ffplay", "opencv"),
        default=os.environ.get("OPENARM_QUEST_CAM_PREVIEW_BACKEND", "auto"),
    )
    args = p.parse_args()

    devices = resolve_devices(parse_devices(args.devices))
    # Even with zero local nodes, MJPEG fallback may still work for FPV.
    if not devices:
        url = mjpeg_url_for_device(f"/dev/video{os.environ.get('OPENARM_QUEST_CAMERA', '16')}")
        if url and args.backend in ("auto", "ffplay"):
            print(f"[cam-preview] no local V4L nodes; trying MJPEG only {url}", flush=True)
            env = gui_env()
            p = spawn_ffplay_url(url, "cam FPV (MJPEG)", env)
            if p is None:
                return 1
            try:
                while p.poll() is None:
                    time.sleep(0.25)
            except KeyboardInterrupt:
                pass
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                except (OSError, ProcessLookupError):
                    p.terminate()
            return 0
        return 1

    print(f"[cam-preview] devices={devices}", flush=True)

    if args.backend in ("auto", "ffplay"):
        rc = run_ffplay(devices, args.width, args.height)
        if rc == 0 or args.backend == "ffplay":
            return rc
        print("[cam-preview] falling back to opencv…", file=sys.stderr, flush=True)

    return run_opencv(devices, args.width, args.height, args.fps)


if __name__ == "__main__":
    raise SystemExit(main())
