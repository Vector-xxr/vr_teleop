#!/usr/bin/env python3
"""KER 动作录制面板：开始/停止 → UDP 通知 ker_real_teleop。

协议（默认 UDP 127.0.0.1:47950）：
  start,<session_name>[,seg]
  stop
  quit          # 关闭面板时发送；遥操进程收到后整机退出
"""

from __future__ import annotations

import argparse
import os
import re
import socket
import time
import tkinter as tk
from pathlib import Path


def default_session_name() -> str:
    return time.strftime("ker_%Y%m%d_%H%M%S")


def sanitize_name(name: str) -> str:
    name = name.strip()
    name = re.sub(r"[^\w.\-]+", "_", name, flags=re.UNICODE)
    name = name.strip("._-")
    return name or default_session_name()


class RecordPanel:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recording = False
        self.session = default_session_name()
        self.seg = 0

        self.root = tk.Tk()
        self.root.title("KER 动作录制")
        self.root.attributes("-topmost", True)
        self.root.geometry("360x220")

        frm = tk.Frame(self.root, padx=12, pady=12)
        frm.pack(fill=tk.BOTH, expand=True)

        tk.Label(frm, text="会话名").pack(anchor="w")
        self.name_var = tk.StringVar(value=self.session)
        self.name_entry = tk.Entry(frm, textvariable=self.name_var, width=36)
        self.name_entry.pack(fill=tk.X, pady=(0, 8))

        self.status = tk.Label(frm, text="未录制", fg="#444")
        self.status.pack(anchor="w", pady=(0, 8))

        self.btn = tk.Button(
            frm,
            text="开始录制",
            bg="#2e7d32",
            fg="white",
            activebackground="#1b5e20",
            height=2,
            command=self.toggle,
        )
        self.btn.pack(fill=tk.X, pady=(0, 8))

        tk.Button(frm, text="退出（停止控制）", command=self.on_quit).pack(fill=tk.X)
        self.root.protocol("WM_DELETE_WINDOW", self.on_quit)

    def send(self, msg: str):
        self.sock.sendto(msg.encode("utf-8"), (self.host, self.port))
        print(f"[ker-panel] → {msg}")

    def toggle(self):
        if not self.recording:
            self.session = sanitize_name(self.name_var.get())
            self.name_var.set(self.session)
            self.name_entry.configure(state=tk.DISABLED)
            self.seg += 1
            self.send(f"start,{self.session},{self.seg}")
            self.recording = True
            self.btn.configure(text="停止录制", bg="#c62828", activebackground="#8e0000")
            self.status.configure(
                text=f"录制中：{self.session} / seg {self.seg}", fg="#c62828"
            )
        else:
            self.send("stop")
            self.recording = False
            self.name_entry.configure(state=tk.NORMAL)
            self.btn.configure(text="开始录制", bg="#2e7d32", activebackground="#1b5e20")
            self.status.configure(
                text=f"已停止：{self.session} / seg {self.seg}", fg="#444"
            )

    def on_quit(self):
        if self.recording:
            self.send("stop")
        # Closing the panel shuts down the whole KER teleop process.
        self.send("quit")
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--host", default=os.getenv("OPENARM_KER_RECORD_HOST", "127.0.0.1"))
    p.add_argument(
        "--port",
        type=int,
        default=int(os.getenv("OPENARM_KER_RECORD_PORT", "47950")),
    )
    args = p.parse_args()
    if not os.environ.get("DISPLAY"):
        os.environ["DISPLAY"] = ":0"
    RecordPanel(args.host, args.port).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
