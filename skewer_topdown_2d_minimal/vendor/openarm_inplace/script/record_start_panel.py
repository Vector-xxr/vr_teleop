#!/usr/bin/env python3
"""Small record-start panel for real C++ leader-follower teleop.

This UI does not send joint references. It only sends a one-shot UDP "start"
message so the C++ teleop process can begin writing the recording CSV.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import socket
import subprocess
import sys
import tkinter as tk
from tkinter.scrolledtext import ScrolledText


class RecordStartPanel:
    def __init__(
        self,
        host: str,
        port: int,
        log_path: str | None = None,
        position_output: str | None = None,
        monitor_port: int = 0,
        arm_side: str = "right_arm",
    ) -> None:
        self.host = host
        self.port = port
        self.log_path = Path(log_path) if log_path else None
        self.position_output = Path(position_output) if position_output else None
        self.monitor_port = monitor_port
        self.arm_side = arm_side
        self.monitor_processes: list[subprocess.Popen] = []
        self.recording = False
        self.segment_index = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.root = tk.Tk()
        self.root.title("OpenArm Recording")
        self.root.resizable(True, True)

        frame = tk.Frame(self.root)
        frame.pack(padx=18, pady=18)

        self.status = tk.StringVar(value="Recording is waiting.")
        tk.Label(frame, textvariable=self.status, width=30).pack(pady=(0, 10))

        self.button = tk.Button(
            frame,
            text="Start Recording",
            width=22,
            height=2,
            command=self.toggle_recording,
        )
        self.button.pack()

        self.dump_button = tk.Button(
            frame,
            text="Save Current Positions",
            width=22,
            height=2,
            command=self.dump_positions,
        )
        self.dump_button.pack(pady=(10, 0))

        self.monitor_button = tk.Button(
            frame,
            text="Open Teleop Monitor",
            width=22,
            height=2,
            command=self.open_monitor,
        )
        self.monitor_button.pack(pady=(10, 0))
        if self.monitor_port <= 0:
            self.monitor_button.configure(state="disabled")

        self.output = ScrolledText(frame, width=80, height=6, wrap=tk.WORD)
        self.output.pack(pady=(10, 0), fill=tk.BOTH, expand=True)
        initial_text = (
            f"Position snapshots will be appended to:\n{self.position_output}\n"
            if self.position_output
            else "No position output file was configured.\n"
        )
        self.output.insert(tk.END, initial_text)
        self.output.configure(state="disabled")

        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def toggle_recording(self) -> None:
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self) -> None:
        self.segment_index += 1
        self.sock.sendto(b"start", (self.host, self.port))
        self.recording = True
        self.status.set(f"Recording segment {self.segment_index:02d} started.")
        self.button.configure(text=f"Stop Recording ({self.segment_index:02d})")

    def stop_recording(self) -> None:
        self.sock.sendto(b"stop", (self.host, self.port))
        self.recording = False
        self.status.set(f"Recording segment {self.segment_index:02d} stopped.")
        self.button.configure(text="Start Recording")

    def dump_positions(self) -> None:
        self.sock.sendto(b"dump", (self.host, self.port))
        if self.position_output:
            self.status.set(f"Saving position snapshot to {self.position_output}")
        else:
            self.status.set("Position dump requested.")
        self.root.after(250, self.refresh_position_output)
        self.root.after(800, self.refresh_position_output)

    def open_monitor(self) -> None:
        self.monitor_processes = [
            process for process in self.monitor_processes if process.poll() is None
        ]
        if self.monitor_processes:
            self.status.set("Monitor panel is already open from this recording panel.")
            return
        script_path = Path(__file__).with_name("teleop_monitor_panel.py")
        right_port = self.monitor_port if self.arm_side != "left_arm" else 0
        left_port = self.monitor_port if self.arm_side == "left_arm" else 0
        args = [
            sys.executable,
            str(script_path),
            "--right-port",
            str(right_port),
            "--left-port",
            str(left_port),
        ]
        try:
            process = subprocess.Popen(args)
        except OSError as exc:
            self.status.set(f"Failed to open monitor: {exc}")
            return
        self.monitor_processes.append(process)
        self.status.set(f"Opened monitor panel on port {self.monitor_port}.")

    def refresh_position_output(self) -> None:
        if self.position_output:
            lines = [
                f"Saved snapshots: {self.position_output}",
                "",
                *self.latest_position_lines(self.position_output, limit=6),
            ]
            if len(lines) == 2:
                lines.append("No saved position snapshots yet.")
            text = "\n".join(lines) + "\n"
        elif self.log_path is None:
            text = "No log path was provided to the panel.\n"
        else:
            latest = self.latest_position_line(self.log_path)
            text = (latest or f"No [positions] line found in {self.log_path}") + "\n"

        self.output.configure(state="normal")
        self.output.delete("1.0", tk.END)
        self.output.insert(tk.END, text)
        self.output.configure(state="disabled")

    @staticmethod
    def latest_position_line(path: Path) -> str | None:
        try:
            for line in reversed(path.read_text(errors="replace").splitlines()):
                if "[positions]" in line:
                    return line
        except OSError:
            return None
        return None

    @staticmethod
    def latest_position_lines(path: Path, limit: int) -> list[str]:
        try:
            lines = [
                line for line in path.read_text(errors="replace").splitlines()
                if "[positions]" in line
            ]
        except OSError:
            return []
        return lines[-limit:]

    def close(self) -> None:
        self.sock.close()
        for process in self.monitor_processes:
            if process.poll() is None:
                process.terminate()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    parser = argparse.ArgumentParser(description="Start-record button for OpenArm real teleop.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=47920)
    parser.add_argument("--log")
    parser.add_argument("--position-output")
    parser.add_argument("--monitor-port", type=int, default=0)
    parser.add_argument("--arm-side", default="right_arm")
    args = parser.parse_args()
    RecordStartPanel(
        args.host,
        args.port,
        args.log,
        args.position_output,
        args.monitor_port,
        args.arm_side,
    ).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
