#!/usr/bin/env python3
"""Single record-start panel for dual-arm real C++ teleop."""

from __future__ import annotations

import argparse
from pathlib import Path
import socket
import subprocess
import sys
import tkinter as tk
from tkinter.scrolledtext import ScrolledText


class DualRecordStartPanel:
    def __init__(
        self,
        host: str,
        right_port: int,
        left_port: int,
        right_log: str | None = None,
        left_log: str | None = None,
        position_output: str | None = None,
        monitor_right_port: int = 0,
        monitor_left_port: int = 0,
    ) -> None:
        self.host = host
        self.ports = [("right", right_port), ("left", left_port)]
        self.logs = [("right", Path(right_log)) if right_log else None,
                     ("left", Path(left_log)) if left_log else None]
        self.position_output = Path(position_output) if position_output else None
        self.monitor_right_port = monitor_right_port
        self.monitor_left_port = monitor_left_port
        self.monitor_processes: list[subprocess.Popen] = []
        self.recording = False
        self.segment_index = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.root = tk.Tk()
        self.root.title("OpenArm Dual Recording")
        self.root.resizable(True, True)

        frame = tk.Frame(self.root)
        frame.pack(padx=18, pady=18)

        self.status = tk.StringVar(
            value=f"Waiting: right:{right_port}, left:{left_port}"
        )
        tk.Label(frame, textvariable=self.status, width=42).pack(pady=(0, 10))

        self.button = tk.Button(
            frame,
            text="Start Both Recordings",
            width=26,
            height=2,
            command=self.toggle_recording,
        )
        self.button.pack()

        self.dump_button = tk.Button(
            frame,
            text="Save Current Positions",
            width=26,
            height=2,
            command=self.dump_positions,
        )
        self.dump_button.pack(pady=(10, 0))

        self.monitor_button = tk.Button(
            frame,
            text="Open Teleop Monitor",
            width=26,
            height=2,
            command=self.open_monitor,
        )
        self.monitor_button.pack(pady=(10, 0))
        if self.monitor_right_port <= 0 and self.monitor_left_port <= 0:
            self.monitor_button.configure(state="disabled")

        self.output = ScrolledText(frame, width=100, height=8, wrap=tk.WORD)
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
        for _, port in self.ports:
            self.sock.sendto(b"start", (self.host, port))
        ports = ", ".join(f"{name}:{port}" for name, port in self.ports)
        self.recording = True
        self.status.set(f"Recording segment {self.segment_index:02d} started: {ports}")
        self.button.configure(text=f"Stop Both Recordings ({self.segment_index:02d})")

    def stop_recording(self) -> None:
        for _, port in self.ports:
            self.sock.sendto(b"stop", (self.host, port))
        self.recording = False
        self.status.set(f"Recording segment {self.segment_index:02d} stopped.")
        self.button.configure(text="Start Both Recordings")

    def dump_positions(self) -> None:
        for _, port in self.ports:
            self.sock.sendto(b"dump", (self.host, port))
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
        args = [
            sys.executable,
            str(script_path),
            "--right-port",
            str(self.monitor_right_port),
            "--left-port",
            str(self.monitor_left_port),
        ]
        try:
            process = subprocess.Popen(args)
        except OSError as exc:
            self.status.set(f"Failed to open monitor: {exc}")
            return
        self.monitor_processes.append(process)
        self.status.set(
            "Opened monitor panel "
            f"(right={self.monitor_right_port}, left={self.monitor_left_port})."
        )

    def refresh_position_output(self) -> None:
        if self.position_output:
            lines = [
                f"Saved snapshots: {self.position_output}",
                "",
                *self.latest_position_lines(self.position_output, limit=8),
            ]
            if len(lines) == 2:
                lines.append("No saved position snapshots yet.")
        else:
            lines = []
            for item in self.logs:
                if item is None:
                    continue
                name, path = item
                latest = self.latest_position_line(path)
                if latest:
                    lines.append(f"{name}: {latest}")
                else:
                    lines.append(f"{name}: no [positions] line found in {path}")
            if not lines:
                lines.append("No log paths were provided to the panel.")

        self.output.configure(state="normal")
        self.output.delete("1.0", tk.END)
        self.output.insert(tk.END, "\n".join(lines) + "\n")
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
    parser = argparse.ArgumentParser(description="Start both OpenArm real teleop recordings.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--right-port", type=int, default=47920)
    parser.add_argument("--left-port", type=int, default=47921)
    parser.add_argument("--right-log")
    parser.add_argument("--left-log")
    parser.add_argument("--position-output")
    parser.add_argument("--monitor-right-port", type=int, default=0)
    parser.add_argument("--monitor-left-port", type=int, default=0)
    args = parser.parse_args()
    DualRecordStartPanel(
        args.host,
        args.right_port,
        args.left_port,
        args.right_log,
        args.left_log,
        args.position_output,
        args.monitor_right_port,
        args.monitor_left_port,
    ).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
