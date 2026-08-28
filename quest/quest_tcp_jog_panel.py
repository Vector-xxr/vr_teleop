#!/usr/bin/env python3
"""Quest 双臂末端 TCP 微调面板：左右臂各「上下左右」共 8 键 + 双手相对锁定 + 主从画圆。

按住按钮 → 末端沿该方向匀速持续移动；松开即停。
无需按 Quest grip。

方向为机器人机体坐标系 openarm_body_link0：
  上=+Z  下=-Z  左=+Y  右=-Y（X 为前方；面板无前后键）。

「双手相对锁定」：冻结当前左↔右 TCP 相对位姿；右手 VR 只跟位置（姿态锁死），
左手自动跟随右手保持相对距离/角度不变。

主从锁定后可「画圆」：以当前右臂 TCP 为圆心，水平面（机器人 XY）转一整圈后停；
半径单位 mm；一圈时长固定（默认 8 s，线速度随半径增大）；左臂仍相对跟随。

命令经 UDP 发送到 quest_cartesian_control（端口默认 47931）：
  hold,<left|right>,<up|down|left|right>
  release,<left|right>,<up|down|left|right>
  couple,toggle|on|off
  circle,start,<radius_mm>
  circle,stop
兼容旧协议（单步）：
  nudge,<left|right>,<up|down|left|right>
"""

from __future__ import annotations

import argparse
import socket
import tkinter as tk


class QuestTcpJogPanel:
    def __init__(self, host: str, port: int, speed_mms: float) -> None:
        self.host = host
        self.port = port
        self.speed_mms = speed_mms
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # (side, direction) currently held
        self.held: set[tuple[str, str]] = set()
        self.couple_on = False
        self.circle_on = False

        self.root = tk.Tk()
        self.root.title("Quest TCP 微调")
        self.root.resizable(False, False)

        outer = tk.Frame(self.root)
        outer.pack(padx=16, pady=14)

        self.status = tk.StringVar(
            value=f"按住滑动 · {speed_mms:.0f} mm/s · UDP {host}:{port}"
        )
        tk.Label(outer, textvariable=self.status).pack(pady=(0, 10))

        row = tk.Frame(outer)
        row.pack()

        self._arm_block(row, "左臂", "left").pack(side=tk.LEFT, padx=12)
        self._arm_block(row, "右臂", "right").pack(side=tk.LEFT, padx=12)

        couple_row = tk.Frame(outer)
        couple_row.pack(pady=(12, 0))
        self.couple_btn = tk.Button(
            couple_row,
            text="双手相对锁定：关",
            width=22,
            height=2,
            command=self._toggle_couple,
        )
        self.couple_btn.pack()

        circle_row = tk.Frame(outer)
        circle_row.pack(pady=(10, 0))
        tk.Label(circle_row, text="半径(mm)").pack(side=tk.LEFT)
        self.radius_var = tk.StringVar(value="50")
        tk.Entry(circle_row, textvariable=self.radius_var, width=8).pack(
            side=tk.LEFT, padx=(6, 10)
        )
        self.circle_start_btn = tk.Button(
            circle_row, text="画圆", width=8, command=self._start_circle
        )
        self.circle_start_btn.pack(side=tk.LEFT, padx=2)
        self.circle_stop_btn = tk.Button(
            circle_row, text="停止", width=8, command=self._stop_circle
        )
        self.circle_stop_btn.pack(side=tk.LEFT, padx=2)

        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<FocusOut>", self._release_all)

    def _arm_block(self, parent: tk.Widget, title: str, side: str) -> tk.Frame:
        box = tk.LabelFrame(parent, text=title, padx=10, pady=8)
        self._make_btn(box, "上", side, "up").grid(row=0, column=1, pady=2)
        self._make_btn(box, "左", side, "left").grid(row=1, column=0, padx=2)
        self._make_btn(box, "右", side, "right").grid(row=1, column=2, padx=2)
        self._make_btn(box, "下", side, "down").grid(row=2, column=1, pady=2)
        return box

    def _make_btn(self, parent: tk.Widget, label: str, side: str, direction: str) -> tk.Button:
        btn = tk.Button(parent, text=label, width=6, height=2)
        btn.bind("<ButtonPress-1>", lambda _e, s=side, d=direction: self._press(s, d))
        btn.bind("<ButtonRelease-1>", lambda _e, s=side, d=direction: self._release(s, d))
        return btn

    def _send(self, msg: str) -> None:
        self.sock.sendto(msg.encode("ascii"), (self.host, self.port))

    def _press(self, side: str, direction: str) -> None:
        key = (side, direction)
        if key in self.held:
            return
        self.held.add(key)
        self._send(f"hold,{side},{direction}")
        self._update_status()

    def _release(self, side: str, direction: str) -> None:
        key = (side, direction)
        if key not in self.held:
            return
        self.held.discard(key)
        self._send(f"release,{side},{direction}")
        self._update_status()

    def _release_all(self, _event=None) -> None:
        for side, direction in list(self.held):
            self._send(f"release,{side},{direction}")
        self.held.clear()
        self._update_status()

    def _toggle_couple(self) -> None:
        self.couple_on = not self.couple_on
        if not self.couple_on and self.circle_on:
            self._stop_circle()
        self._send("couple,on" if self.couple_on else "couple,off")
        self.couple_btn.configure(
            text=("双手相对锁定：开" if self.couple_on else "双手相对锁定：关"),
            relief=(tk.SUNKEN if self.couple_on else tk.RAISED),
        )
        self._update_status()

    def _parse_radius_mm(self) -> float | None:
        try:
            r = float(self.radius_var.get().strip())
        except ValueError:
            return None
        if not (1.0 <= r <= 500.0):
            return None
        return r

    def _start_circle(self) -> None:
        if not self.couple_on:
            self.status.set("需先开双手相对锁定，再画圆")
            return
        radius_mm = self._parse_radius_mm()
        if radius_mm is None:
            self.status.set("半径无效（请输入 1–500 mm）")
            return
        self._send(f"circle,start,{radius_mm:g}")
        self.circle_on = True
        self._update_status()

    def _stop_circle(self) -> None:
        self._send("circle,stop")
        self.circle_on = False
        self._update_status()

    def _update_status(self) -> None:
        couple = "· 主从锁定" if self.couple_on else ""
        circle = "· 画圆中" if self.circle_on else ""
        if not self.held:
            self.status.set(
                f"按住滑动 · {self.speed_mms:.0f} mm/s · UDP {self.host}:{self.port}"
                f" {couple}{circle}"
            )
            return
        parts = [f"{s}/{d}" for s, d in sorted(self.held)]
        self.status.set(
            f"按住中: {', '.join(parts)} · {self.speed_mms:.0f} mm/s {couple}{circle}"
        )

    def close(self) -> None:
        self._release_all()
        if self.circle_on:
            self._send("circle,stop")
            self.circle_on = False
        if self.couple_on:
            self._send("couple,off")
            self.couple_on = False
        self.sock.close()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    p = argparse.ArgumentParser(description="Quest TCP hold-to-jog panel (8 buttons + couple + circle)")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=47931)
    p.add_argument(
        "--speed",
        type=float,
        default=20.0,
        help="tip speed while held (mm/s); must match controller OPENARM_QUEST_JOG_SPEED",
    )
    # Legacy alias kept so old launch scripts still parse.
    p.add_argument("--step", type=float, default=None, help=argparse.SUPPRESS)
    args = p.parse_args()
    speed = args.speed
    QuestTcpJogPanel(args.host, args.port, speed).run()


if __name__ == "__main__":
    main()
