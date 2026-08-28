#!/usr/bin/env python3
"""Tk joint panel for the C++ OpenArm teleop process.

This script is intentionally only a UI/input source. It sends CSV UDP packets
to the C++ teleop process, which owns the real CAN/control loop.
"""

from __future__ import annotations

import argparse
import socket
import tkinter as tk

INITIAL = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
MIN_LIMIT = [-2.0944, -1.5708, -1.5708, 0.0, -1.5708, -1.5708, -1.5708, 0.0]
MAX_LIMIT = [2.0944, 3.1416, 1.5708, 3.1416, 1.5708, 1.5708, 1.5708, 0.044]


class JointPanel:
    def __init__(self, host: str, port: int, hz: float, arms: str) -> None:
        self.host = host
        self.port = port
        self.period_ms = max(10, int(1000.0 / hz))
        self.arms = ["right"] if arms == "single" else ["left", "right"]
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.root = tk.Tk()
        self.root.title("OpenArm C++ Teleop Joint Panel")
        self.vars: list[tk.DoubleVar] = []

        outer = tk.Frame(self.root)
        outer.pack(padx=10, pady=10, fill="both")

        for col, arm in enumerate(self.arms):
            frame = tk.LabelFrame(
                outer, text=f"{arm.capitalize()} arm reference", padx=8, pady=6
            )
            frame.grid(row=0, column=col, padx=5, sticky="nsew")
            outer.grid_columnconfigure(col, weight=1)
            for i in range(8):
                label = f"{arm[0].upper()} J{i + 1} (rad)" if i < 7 else f"{arm[0].upper()} Gripper (m)"
                var = tk.DoubleVar(value=INITIAL[i])
                tk.Scale(
                    frame,
                    from_=MIN_LIMIT[i],
                    to=MAX_LIMIT[i],
                    resolution=0.005 if i < 7 else 0.001,
                    orient="horizontal",
                    length=360,
                    label=label,
                    variable=var,
                ).pack(fill="x")
                self.vars.append(var)

        controls = tk.Frame(self.root)
        controls.pack(pady=6)
        tk.Button(controls, text="Home", width=12, command=self.home).pack(side="left", padx=4)
        tk.Button(controls, text="Close", width=12, command=self.close).pack(side="left", padx=4)

        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def home(self) -> None:
        for idx, var in enumerate(self.vars):
            var.set(INITIAL[idx % len(INITIAL)])

    def send(self) -> None:
        values = [var.get() for var in self.vars]
        payload = ",".join(f"{value:.6f}" for value in values).encode("ascii")
        self.sock.sendto(payload, (self.host, self.port))
        self.root.after(self.period_ms, self.send)

    def close(self) -> None:
        self.sock.close()
        self.root.destroy()

    def run(self) -> None:
        self.root.after(50, self.send)
        self.root.mainloop()


def main() -> int:
    parser = argparse.ArgumentParser(description="Joint slider panel for C++ OpenArm teleop.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=47910)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--arms", choices=("single", "bimanual"), default="single")
    args = parser.parse_args()
    JointPanel(args.host, args.port, args.hz, args.arms).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
