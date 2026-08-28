#!/usr/bin/env python3
"""Quest teleop 动作录制面板。

按钮：
  1. 动作录制开始 / 停止（首次开始时才在控制端创建会话目录）
  2. 预览摄像头（单独弹窗；不依赖 opencv，优先 ffplay）
  3. 当前姿态保存
  4. 退出

协议：
  start,<session_name>
  stop
  dump[,<session_name>]
  shutdown

摄像头：
  OPENARM_QUEST_RECORD_CAMS=8,16          优先设备
  OPENARM_QUEST_RECORD_CAM_PREVIEW=0      关闭录制时自动预览
  OPENARM_QUEST_RECORD_CAM_AUTOSTART=0    关闭面板启动时自动预览（默认开）
"""

from __future__ import annotations

import argparse
import glob
import os
import pwd
import re
import signal
import socket
import subprocess
import sys
import time
import tkinter as tk


def default_session_name() -> str:
    return time.strftime("quest_%Y%m%d_%H%M%S")


def sanitize_name(name: str) -> str:
    name = name.strip()
    name = re.sub(r"[^\w.\-]+", "_", name, flags=re.UNICODE)
    name = name.strip("._-")
    return name or default_session_name()


def env_truthy(name: str, default: str = "1") -> bool:
    return os.environ.get(name, default).strip().lower() in ("1", "true", "yes", "on")


def _find_xauthority() -> str | None:
    candidates: list[str] = []
    env_xa = os.environ.get("XAUTHORITY")
    if env_xa:
        candidates.append(env_xa)
    # Common desktop locations (GNOME on Wayland uses mutter-Xwaylandauth.*).
    for uid in (os.geteuid(), 1000):
        candidates.extend(sorted(glob.glob(f"/run/user/{uid}/.mutter-Xwaylandauth.*")))
        candidates.append(f"/run/user/{uid}/gdm/Xauthority")
    try:
        home = pwd.getpwuid(os.geteuid()).pw_dir
        candidates.append(os.path.join(home, ".Xauthority"))
    except KeyError:
        pass
    candidates.append(os.path.expanduser("~/.Xauthority"))
    candidates.append("/home/lan/.Xauthority")
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    return None


def gui_env() -> dict[str, str]:
    env = os.environ.copy()
    if not env.get("DISPLAY"):
        env["DISPLAY"] = ":0"
    xa = _find_xauthority()
    if xa:
        env["XAUTHORITY"] = xa
    if "SDL_VIDEODRIVER" not in env:
        env["SDL_VIDEODRIVER"] = "x11"
    # Avoid PulseAudio spam when root / no session bus.
    env.setdefault("SDL_AUDIODRIVER", "dummy")
    return env


class QuestActionRecordPanel:
    def __init__(self, host: str, port: int, session_name: str = "") -> None:
        self.host = host
        self.port = port
        self.recording = False
        self.segment = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cam_proc: subprocess.Popen | None = None
        self.cam_log_path = "/tmp/openarm_cam_preview.log"
        self.quest_dir = os.path.dirname(os.path.abspath(__file__))

        self.root = tk.Tk()
        self.root.title("Quest 动作录制")
        self.root.resizable(False, False)

        frame = tk.Frame(self.root)
        frame.pack(padx=20, pady=18)

        name_row = tk.Frame(frame)
        name_row.pack(fill=tk.X, pady=(0, 10))
        tk.Label(name_row, text="会话名").pack(side=tk.LEFT)
        self.name_var = tk.StringVar(value=session_name or default_session_name())
        self.name_entry = tk.Entry(name_row, textvariable=self.name_var, width=28)
        self.name_entry.pack(side=tk.LEFT, padx=(8, 0))

        cams = os.environ.get("OPENARM_QUEST_RECORD_CAMS", "8,16")
        cam_note = f"摄像头 {cams}：面板打开后自动预览（也可点按钮）"
        if not env_truthy("OPENARM_QUEST_RECORD_CAM_AUTOSTART", "1"):
            cam_note = f"摄像头 {cams}：点「预览摄像头」或开始录制"
        self.status = tk.StringVar(
            value=f"待命 · UDP {host}:{port}（开始录制后才建目录）\n{cam_note}"
        )
        tk.Label(frame, textvariable=self.status, width=44, wraplength=340, justify=tk.LEFT).pack(
            pady=(0, 12)
        )

        self.btn_record = tk.Button(
            frame,
            text="动作录制开始",
            width=24,
            height=2,
            command=self.toggle_record,
        )
        self.btn_record.pack()

        self.btn_cams = tk.Button(
            frame,
            text="预览摄像头",
            width=24,
            height=2,
            command=self.toggle_camera_preview_button,
        )
        self.btn_cams.pack(pady=(10, 0))

        self.btn_save = tk.Button(
            frame,
            text="当前姿态保存",
            width=24,
            height=2,
            command=self.save_pose,
        )
        self.btn_save.pack(pady=(10, 0))

        self.btn_exit = tk.Button(
            frame,
            text="退出",
            width=24,
            height=2,
            command=self.close,
        )
        self.btn_exit.pack(pady=(10, 0))

        self.root.protocol("WM_DELETE_WINDOW", self.close)

        # Auto-open cams shortly after the panel maps (user expects windows with teleop).
        if env_truthy("OPENARM_QUEST_RECORD_CAM_AUTOSTART", "1"):
            self.root.after(600, self._autostart_camera_preview)

    def _cam_running(self) -> bool:
        return self.cam_proc is not None and self.cam_proc.poll() is None

    def _autostart_camera_preview(self) -> None:
        if self._cam_running():
            return
        self._start_camera_preview(force=True)
        if self._cam_running():
            self.btn_cams.configure(text="关闭摄像头预览")

    def toggle_camera_preview_button(self) -> None:
        if self._cam_running():
            self._stop_camera_preview()
            self.btn_cams.configure(text="预览摄像头")
            self.status.set("已关闭摄像头预览")
            return
        self._start_camera_preview(force=True)
        self.btn_cams.configure(text="关闭摄像头预览")

    def _start_camera_preview(self, force: bool = False) -> None:
        if self._cam_running():
            return
        self._stop_camera_preview()
        if not force and not env_truthy("OPENARM_QUEST_RECORD_CAM_PREVIEW", "1"):
            return
        script = os.path.join(self.quest_dir, "preview_dual_v4l_cams.py")
        if not os.path.isfile(script):
            print(f"[record-panel] missing {script}", file=sys.stderr, flush=True)
            self.status.set("摄像头预览脚本缺失")
            return
        devices = os.environ.get("OPENARM_QUEST_RECORD_CAMS", "8,16")
        env = gui_env()
        try:
            log_f = open(self.cam_log_path, "w", encoding="utf-8")
            try:
                os.fchmod(log_f.fileno(), 0o666)
            except OSError:
                pass
            self.cam_proc = subprocess.Popen(
                [sys.executable, "-u", script, "--devices", devices],
                cwd=self.quest_dir,
                start_new_session=True,
                stdout=log_f,
                stderr=subprocess.STDOUT,
                env=env,
            )
            log_f.close()
            print(
                f"[record-panel] camera preview pid={self.cam_proc.pid} "
                f"devices={devices} log={self.cam_log_path} "
                f"DISPLAY={env.get('DISPLAY')} XAUTHORITY={env.get('XAUTHORITY')}",
                flush=True,
            )
            self.status.set(f"正在打开摄像头…（日志 {self.cam_log_path}）")
            self.root.after(1500, self._check_camera_preview)
        except OSError as exc:
            print(f"[record-panel] camera preview failed: {exc}", file=sys.stderr, flush=True)
            self.cam_proc = None
            self.status.set(f"摄像头预览启动失败: {exc}")

    def _check_camera_preview(self) -> None:
        proc = self.cam_proc
        if proc is None:
            return
        rc = proc.poll()
        if rc is None:
            cams = os.environ.get("OPENARM_QUEST_RECORD_CAMS", "8,16")
            if self.recording:
                self.status.set(f"录制中 · 段 {self.segment:02d} · 摄像头已开({cams})")
            else:
                self.status.set(f"摄像头预览运行中({cams}) · 日志 {self.cam_log_path}")
            self.btn_cams.configure(text="关闭摄像头预览")
            return
        msg = f"摄像头预览已退出(code={rc})"
        if os.path.isfile(self.cam_log_path):
            try:
                with open(self.cam_log_path, encoding="utf-8", errors="replace") as f:
                    lines = [ln.strip() for ln in f.readlines() if ln.strip()]
                if lines:
                    msg = f"{msg}: {lines[-1][:120]}"
            except OSError:
                pass
        print(f"[record-panel] {msg}", file=sys.stderr, flush=True)
        self.status.set(msg)
        self.cam_proc = None
        self.btn_cams.configure(text="预览摄像头")

    def _stop_camera_preview(self) -> None:
        proc = self.cam_proc
        self.cam_proc = None
        if proc is None:
            return
        try:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=1.0)
        except (OSError, ProcessLookupError):
            pass

    def toggle_record(self) -> None:
        if self.recording:
            self.sock.sendto(b"stop", (self.host, self.port))
            self.recording = False
            # Keep camera windows open after stop — user still wants preview.
            self.status.set(f"已停止录制段 {self.segment:02d}")
            self.btn_record.configure(text="动作录制开始")
            self.name_entry.configure(state=tk.NORMAL)
            return

        session = sanitize_name(self.name_var.get())
        self.name_var.set(session)
        self.segment += 1
        msg = f"start,{session}".encode("utf-8")
        self.sock.sendto(msg, (self.host, self.port))
        self.recording = True
        self.name_entry.configure(state=tk.DISABLED)
        if not self._cam_running() and env_truthy("OPENARM_QUEST_RECORD_CAM_PREVIEW", "1"):
            self._start_camera_preview(force=True)
            self.btn_cams.configure(text="关闭摄像头预览")
        self.status.set(f"录制中 · {session}/{self.segment:02d}")
        self.btn_record.configure(text=f"停止录制 ({self.segment:02d})")

    def save_pose(self) -> None:
        session = sanitize_name(self.name_var.get())
        self.name_var.set(session)
        msg = f"dump,{session}".encode("utf-8")
        self.sock.sendto(msg, (self.host, self.port))
        self.status.set(f"已请求保存姿态 → {session}")

    def close(self) -> None:
        if getattr(self, "_closing", False):
            return
        self._closing = True
        try:
            if self.recording:
                self.sock.sendto(b"stop", (self.host, self.port))
                self.recording = False
            self._stop_camera_preview()
            self.sock.sendto(b"shutdown", (self.host, self.port))
            self.status.set("已请求结束控制（阻尼释放中）…")
            self.root.update_idletasks()
            time.sleep(0.15)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    p = argparse.ArgumentParser(description="Quest teleop action record panel")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=47930)
    p.add_argument(
        "--session-name",
        default="",
        help="默认会话名（可在面板里改）；也可用环境变量 OPENARM_QUEST_RECORD_NAME",
    )
    args = p.parse_args()

    name = args.session_name or os.environ.get("OPENARM_QUEST_RECORD_NAME", "")
    QuestActionRecordPanel(args.host, args.port, name).run()


if __name__ == "__main__":
    main()
