#!/usr/bin/env python3
"""Realtime 16-channel motor torque panel for dual real replay."""

from __future__ import annotations

import argparse
import csv
import socket
import time
import tkinter as tk
from collections import deque
from pathlib import Path


class TorqueTelemetryPanel:
    def __init__(
        self,
        host: str,
        port: int,
        window_s: float,
        torque_limit: float,
        output_path: str | None,
        title: str | None,
    ) -> None:
        self.host = host
        self.port = port
        self.window_s = window_s
        self.torque_limit = torque_limit
        self.output_path = Path(output_path) if output_path else None
        self.output_file = None
        self.output_writer = None
        self.log_file = None
        self.log_path = (
            self.output_path.with_suffix(self.output_path.suffix + ".log")
            if self.output_path is not None
            else Path("torque_telemetry_disconnect.log")
        )
        self.start_wall = time.monotonic()
        self.last_packet_wall: float | None = None
        self.packet_count = 0
        self.show_full_history = False
        self.segment_index = 1
        self.time_offset = 0.0
        self.last_raw_t: float | None = None
        self.last_display_t = 0.0
        self.left_gripper_contact_hold = False
        self.right_gripper_contact_hold = False
        self.left_gripper_slip_boost = False
        self.right_gripper_slip_boost = False
        self.reserved_alert_mask = 0
        self.error_codes = [0 for _ in range(16)]
        self.mos_temps = [0 for _ in range(16)]
        self.rotor_temps = [0 for _ in range(16)]
        self.full_samples: list[list[tuple[float, float]]] = [
            [] for _ in range(16)
        ]
        self.samples: list[deque[tuple[float, float]]] = [
            deque() for _ in range(16)
        ]

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind((host, port))
        self.sock.setblocking(False)

        if self.output_path is not None:
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self.output_file = self.output_path.open("w", newline="")
            self.output_writer = csv.writer(self.output_file)
            self.output_writer.writerow(
                [
                    "time",
                    "segment",
                    "source_time",
                    "left_gripper_mode",
                    "right_gripper_mode",
                    "reserved_alert_mask",
                ]
                + [self.channel_name(i) for i in range(16)]
                + [f"{self.channel_name(i)}_error_code" for i in range(16)]
                + [f"{self.channel_name(i)}_t_mos_c" for i in range(16)]
                + [f"{self.channel_name(i)}_t_rotor_c" for i in range(16)]
            )
            self.output_file.flush()
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = self.log_path.open("a")

        self.root = tk.Tk()
        title_prefix = title if title else "OpenArm Torque Telemetry"
        self.root.title(f"{title_prefix} - UDP {host}:{port}")
        self.root.geometry("1200x800")

        output_text = f" Saving CSV: {self.output_path}" if self.output_path else ""
        self.status = tk.StringVar(value=f"Waiting for torque telemetry...{output_text}")
        tk.Label(self.root, textvariable=self.status, anchor="w").pack(fill=tk.X, padx=8, pady=4)

        controls = tk.Frame(self.root)
        controls.pack(fill=tk.X, padx=8, pady=(0, 4))
        self.view_button = tk.Button(
            controls,
            text="Show Full History",
            command=self.toggle_view,
        )
        self.view_button.pack(side=tk.LEFT)

        self.canvas = tk.Canvas(self.root, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def close(self) -> None:
        self.sock.close()
        if self.output_file is not None:
            self.output_file.flush()
            self.output_file.close()
        if self.log_file is not None:
            self.log_file.flush()
            self.log_file.close()
        self.root.destroy()

    def run(self) -> None:
        self.root.after(20, self.update)
        self.root.mainloop()

    def update(self) -> None:
        self.receive_packets()
        self.maybe_switch_to_full_history()
        self.draw()
        self.root.after(20, self.update)

    def toggle_view(self) -> None:
        self.show_full_history = not self.show_full_history
        self.view_button.configure(
            text="Show Realtime Window" if self.show_full_history else "Show Full History"
        )
        self.draw()

    def maybe_switch_to_full_history(self) -> None:
        if self.show_full_history or self.last_packet_wall is None:
            return
        if time.monotonic() - self.last_packet_wall > 1.5 and any(self.full_samples):
            self.show_full_history = True
            self.view_button.configure(text="Show Realtime Window")
            self.status.set("Telemetry stopped; showing full history.")

    def receive_packets(self) -> None:
        received = 0
        while True:
            try:
                packet, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            text = packet.decode("utf-8", errors="replace").strip()
            parts = text.split(",")
            if len(parts) not in (17, 19, 21, 22) and len(parts) < 70:
                continue
            try:
                raw_t = float(parts[0])
                reserved_alert_mask = 0
                if len(parts) >= 70:
                    self.left_gripper_contact_hold = bool(int(parts[1]))
                    self.right_gripper_contact_hold = bool(int(parts[2]))
                    self.left_gripper_slip_boost = bool(int(parts[3]))
                    self.right_gripper_slip_boost = bool(int(parts[4]))
                    values = [float(value) for value in parts[5:21]]
                    reserved_alert_mask = int(parts[21])
                    self.error_codes = [int(value) for value in parts[22:38]]
                    self.mos_temps = [int(value) for value in parts[38:54]]
                    self.rotor_temps = [int(value) for value in parts[54:70]]
                elif len(parts) == 22:
                    self.left_gripper_contact_hold = bool(int(parts[1]))
                    self.right_gripper_contact_hold = bool(int(parts[2]))
                    self.left_gripper_slip_boost = bool(int(parts[3]))
                    self.right_gripper_slip_boost = bool(int(parts[4]))
                    values = [float(value) for value in parts[5:21]]
                    reserved_alert_mask = int(parts[21])
                elif len(parts) == 21:
                    self.left_gripper_contact_hold = bool(int(parts[1]))
                    self.right_gripper_contact_hold = bool(int(parts[2]))
                    self.left_gripper_slip_boost = bool(int(parts[3]))
                    self.right_gripper_slip_boost = bool(int(parts[4]))
                    values = [float(value) for value in parts[5:]]
                elif len(parts) == 19:
                    self.left_gripper_contact_hold = bool(int(parts[1]))
                    self.right_gripper_contact_hold = bool(int(parts[2]))
                    self.left_gripper_slip_boost = False
                    self.right_gripper_slip_boost = False
                    values = [float(value) for value in parts[3:]]
                else:
                    self.left_gripper_slip_boost = False
                    self.right_gripper_slip_boost = False
                    values = [float(value) for value in parts[1:]]
            except ValueError:
                continue

            t = self.display_time(raw_t)
            self.handle_reserved_alert_mask(reserved_alert_mask)
            for idx, value in enumerate(values):
                self.samples[idx].append((t, value))
                self.full_samples[idx].append((t, value))
            if self.output_writer is not None and self.output_file is not None:
                self.output_writer.writerow(
                    [
                        f"{t:.6f}",
                        self.segment_index,
                        f"{raw_t:.6f}",
                        self.gripper_mode_text(
                            self.left_gripper_contact_hold,
                            self.left_gripper_slip_boost,
                        ),
                        self.gripper_mode_text(
                            self.right_gripper_contact_hold,
                            self.right_gripper_slip_boost,
                        ),
                        f"0x{self.reserved_alert_mask:04x}",
                    ]
                    + [f"{value:.9f}" for value in values]
                    + [str(value) for value in self.error_codes]
                    + [str(value) for value in self.mos_temps]
                    + [str(value) for value in self.rotor_temps]
                )
            self.last_packet_wall = time.monotonic()
            received += 1

        if received:
            self.packet_count += received
            if self.output_file is not None:
                self.output_file.flush()
            latest = max((series[-1][0] for series in self.samples if series), default=0.0)
            output_text = f", saved={self.output_path}" if self.output_path else ""
            view_text = "full history" if self.show_full_history else f"last {self.window_s:.1f}s"
            self.status.set(
                f"Receiving torque telemetry: latest t={latest:.3f}s, samples={self.packet_count}, "
                f"segment={self.segment_index:02d}, view={view_text}, "
                f"left_gripper={self.gripper_mode_text(self.left_gripper_contact_hold, self.left_gripper_slip_boost)}, "
                f"right_gripper={self.gripper_mode_text(self.right_gripper_contact_hold, self.right_gripper_slip_boost)}"
                f"{self.fault_status_text()}"
                f"{output_text}"
            )

        cutoff = max((series[-1][0] for series in self.samples if series), default=0.0) - self.window_s
        for series in self.samples:
            while series and series[0][0] < cutoff:
                series.popleft()

    def display_time(self, raw_t: float) -> float:
        if self.last_raw_t is not None and raw_t + 0.05 < self.last_raw_t:
            self.segment_index += 1
            self.time_offset = self.last_display_t + 0.05
            if self.show_full_history:
                self.show_full_history = False
                self.view_button.configure(text="Show Full History")
            self.status.set(f"Detected replay loop segment {self.segment_index:02d}.")
        display_t = self.time_offset + raw_t
        if display_t <= self.last_display_t:
            display_t = self.last_display_t + 1e-6
        self.last_raw_t = raw_t
        self.last_display_t = display_t
        return display_t

    def handle_reserved_alert_mask(self, reserved_alert_mask: int) -> None:
        self.reserved_alert_mask = reserved_alert_mask

    def fault_status_text(self) -> str:
        fault_channels = [
            f"{self.channel_name(idx)}:{code}"
            for idx, code in enumerate(self.error_codes)
            if self.is_fault_code(code)
        ]
        if not fault_channels:
            return ""
        return ", FAULT=" + " ".join(fault_channels)

    def draw(self) -> None:
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        cols = 4
        rows = 4
        pad = 8
        cell_w = width / cols
        cell_h = height / rows

        source = self.full_samples if self.show_full_history else self.samples
        for idx, series in enumerate(source):
            row = idx // cols
            col = idx % cols
            x0 = col * cell_w + pad
            y0 = row * cell_h + pad
            x1 = (col + 1) * cell_w - pad
            y1 = (row + 1) * cell_h - pad
            self.draw_chart(idx, list(series), x0, y0, x1, y1)

    def draw_chart(self, idx: int, series: list[tuple[float, float]], x0: float, y0: float, x1: float, y1: float) -> None:
        fault_code = self.error_codes[idx] if idx < len(self.error_codes) else 0
        fault = self.is_fault_code(fault_code)
        outline = "#ff4136" if fault else "#cccccc"
        width = 3 if fault else 1
        self.canvas.create_rectangle(x0, y0, x1, y1, outline=outline, width=width)
        name = self.channel_name(idx)
        latest = series[-1][1] if series else 0.0
        max_sample = self.max_abs_sample(self.full_samples[idx])
        max_text = ""
        if max_sample is not None:
            _, max_value = max_sample
            max_text = f"  max={max_value:+.3f} Nm"
        self.canvas.create_text(
            x0 + 6,
            y0 + 5,
            anchor="nw",
            text=f"{name}  {latest:+.3f} Nm{max_text}",
            fill="#ff4136" if fault else "#111111",
            font=("TkDefaultFont", 9, "bold"),
        )
        mos_temp = self.mos_temps[idx] if idx < len(self.mos_temps) else 0
        rotor_temp = self.rotor_temps[idx] if idx < len(self.rotor_temps) else 0
        fault_text = (
            f"  FAULT {fault_code}: {self.fault_text(fault_code)}"
            if fault else ""
        )
        self.canvas.create_text(
            x0 + 6,
            y0 + 22,
            anchor="nw",
            text=f"MOS {mos_temp}C  rotor {rotor_temp}C{fault_text}",
            fill="#ff4136" if fault else "#444444",
            font=("TkDefaultFont", 8, "bold" if fault else "normal"),
        )

        plot_x0 = x0 + 42
        plot_x1 = x1 - 8
        plot_y0 = y0 + 42
        plot_y1 = y1 - 24
        plot_mid_y = (plot_y0 + plot_y1) / 2.0
        self.canvas.create_line(plot_x0, plot_mid_y, plot_x1, plot_mid_y, fill="#dddddd")
        self.canvas.create_line(plot_x0, plot_y0, plot_x0, plot_y1, fill="#999999")
        self.canvas.create_line(plot_x0, plot_y1, plot_x1, plot_y1, fill="#999999")

        if len(series) < 2:
            self.draw_axes_labels(plot_x0, plot_y0, plot_x1, plot_y1, 0.0, 0.0, self.torque_limit)
            return

        t_latest = series[-1][0]
        t_start = series[0][0] if self.show_full_history else max(t_latest - self.window_s, series[0][0])
        span = max(t_latest - t_start, 1e-6)
        abs_limit = max(self.torque_limit, max(abs(value) for _, value in series), 1e-6)
        plot_half_h = max((plot_y1 - plot_y0) / 2.0, 1.0)
        self.draw_axes_labels(plot_x0, plot_y0, plot_x1, plot_y1, t_start, t_latest, abs_limit)

        points: list[float] = []
        draw_series = self.decimate_series(series, max(2, int(plot_x1 - plot_x0)))
        for t, value in draw_series:
            x = plot_x0 + ((t - t_start) / span) * (plot_x1 - plot_x0)
            y = plot_mid_y - (value / abs_limit) * plot_half_h
            points.extend([x, y])
        if len(points) >= 4:
            self.canvas.create_line(*points, fill="#0074d9", width=1.5)

        if max_sample is not None and t_start <= max_sample[0] <= t_latest:
            max_t, max_value = max_sample
            max_x = plot_x0 + ((max_t - t_start) / span) * (plot_x1 - plot_x0)
            max_y = plot_mid_y - (max_value / abs_limit) * plot_half_h
            self.canvas.create_oval(max_x - 3, max_y - 3, max_x + 3, max_y + 3, fill="#ff4136", outline="#ff4136")
            self.canvas.create_text(
                max_x + 5,
                max_y - 5,
                anchor="sw",
                text=f"max\n{max_value:+.2f} Nm",
                fill="#ff4136",
                font=("TkDefaultFont", 7, "bold"),
            )

    @staticmethod
    def gripper_mode_text(contact_hold: bool, slip_boost: bool = False) -> str:
        if slip_boost:
            return "force-position/slip-boost"
        return "force-position/contact-hold" if contact_hold else "position"

    @staticmethod
    def max_abs_sample(series: list[tuple[float, float]]) -> tuple[float, float] | None:
        if not series:
            return None
        return max(series, key=lambda item: abs(item[1]))

    @staticmethod
    def decimate_series(
        series: list[tuple[float, float]], max_points: int
    ) -> list[tuple[float, float]]:
        if len(series) <= max_points:
            return series
        stride = max(1, len(series) // max_points)
        decimated = series[::stride]
        if decimated[-1] != series[-1]:
            decimated.append(series[-1])
        return decimated

    def draw_axes_labels(
        self,
        plot_x0: float,
        plot_y0: float,
        plot_x1: float,
        plot_y1: float,
        t_start: float,
        t_end: float,
        torque_limit: float,
    ) -> None:
        mid_y = (plot_y0 + plot_y1) / 2.0
        for y, label in (
            (plot_y0, f"+{torque_limit:.1f}"),
            (mid_y, "0"),
            (plot_y1, f"-{torque_limit:.1f}"),
        ):
            self.canvas.create_line(plot_x0 - 3, y, plot_x0, y, fill="#999999")
            self.canvas.create_text(plot_x0 - 5, y, anchor="e", text=label, fill="#666666", font=("TkDefaultFont", 7))

        for x, label in (
            (plot_x0, f"{t_start:.1f}s"),
            (plot_x1, f"{t_end:.1f}s"),
        ):
            self.canvas.create_line(x, plot_y1, x, plot_y1 + 3, fill="#999999")
            self.canvas.create_text(x, plot_y1 + 5, anchor="n", text=label, fill="#666666", font=("TkDefaultFont", 7))

    @staticmethod
    def channel_name(idx: int) -> str:
        side = "L" if idx < 8 else "R"
        local = idx if idx < 8 else idx - 8
        joint = f"J{local + 1}" if local < 7 else "Grip"
        return f"{side}-{joint}"

    @staticmethod
    def is_fault_code(code: int) -> bool:
        return 8 <= code <= 14

    @staticmethod
    def fault_text(code: int) -> str:
        return {
            8: "over-voltage",
            9: "under-voltage",
            10: "over-current",
            11: "MOS over-temp",
            12: "rotor over-temp",
            13: "communication lost",
            14: "overload",
        }.get(code, "unknown")


def main() -> int:
    parser = argparse.ArgumentParser(description="Realtime OpenArm motor torque panel.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=47940)
    parser.add_argument("--window", type=float, default=10.0)
    parser.add_argument("--torque-limit", type=float, default=20.0)
    parser.add_argument("--output", help="CSV path for saving all 16 torque channels")
    parser.add_argument("--title", help="Window title prefix")
    args = parser.parse_args()
    TorqueTelemetryPanel(
        args.host,
        args.port,
        args.window,
        args.torque_limit,
        args.output,
        args.title,
    ).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
