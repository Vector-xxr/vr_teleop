#!/usr/bin/env python3
"""Leader/follower live telemetry monitor for real teleop."""

from __future__ import annotations

import argparse
import socket
import time
import tkinter as tk
from dataclasses import dataclass, field


JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "Grip"]


@dataclass
class SideState:
    side: str
    last_wall: float | None = None
    last_t: float = 0.0
    packet_count: int = 0
    first_wall: float | None = None
    live_started: bool = False
    gripper_contact_hold_active: bool = False
    gripper_slip_boost_active: bool = False
    leader: list[tuple[float, float, float]] = field(default_factory=list)
    reference: list[tuple[float, float, float]] = field(default_factory=list)
    follower: list[tuple[float, float, float]] = field(default_factory=list)
    last_leader: list[tuple[float, float, float]] = field(default_factory=list)
    max_abs_dq: float = 0.0
    max_abs_delta_q: float = 0.0
    max_abs_ref_error: float = 0.0


class TeleopMonitorPanel:
    def __init__(self, host: str, right_port: int, left_port: int, stale_s: float) -> None:
        self.host = host
        self.stale_s = stale_s
        self.sockets: list[socket.socket] = []
        self.bind_errors: list[str] = []
        self.states = {
            "right_arm": SideState("right_arm"),
            "left_arm": SideState("left_arm"),
        }

        for label, port in (("right", right_port), ("left", left_port)):
            if port <= 0:
                continue
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
            try:
                sock.bind((host, port))
            except OSError as exc:
                sock.close()
                self.bind_errors.append(f"{label} port {port}: {exc}")
                continue
            sock.setblocking(False)
            self.sockets.append(sock)

        self.root = tk.Tk()
        ports_text = f"right={right_port}, left={left_port}"
        self.root.title(f"OpenArm Teleop Monitor - UDP {ports_text}")
        self.root.geometry("1120x720")
        if self.bind_errors:
            status = "Monitor port already in use: " + "; ".join(self.bind_errors)
        else:
            status = f"Waiting for teleop monitor packets... {ports_text}"
        self.status = tk.StringVar(value=status)
        tk.Label(self.root, textvariable=self.status, anchor="w").pack(fill=tk.X, padx=8, pady=4)

        self.text = tk.Text(self.root, font=("TkFixedFont", 10), wrap="none")
        self.text.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def close(self) -> None:
        for sock in self.sockets:
            sock.close()
        self.root.destroy()

    def run(self) -> None:
        self.root.after(50, self.update)
        self.root.mainloop()

    def update(self) -> None:
        self.receive_packets()
        self.draw()
        self.root.after(50, self.update)

    def receive_packets(self) -> None:
        received = 0
        while True:
            any_packet = False
            for sock in self.sockets:
                while True:
                    try:
                        packet, _ = sock.recvfrom(4096)
                    except BlockingIOError:
                        break
                    any_packet = True
                    received += 1
                    self.handle_packet(packet.decode("utf-8", errors="replace").strip())
            if not any_packet:
                break
        if received:
            self.status.set(f"Receiving teleop monitor data, packets +{received}")

    def handle_packet(self, text: str) -> None:
        parts = text.split(",")
        if len(parts) not in (75, 76, 77):
            return
        side = parts[0]
        if side not in self.states:
            return
        try:
            live_started = bool(int(parts[1]))
            if len(parts) == 77:
                gripper_contact_hold_active = bool(int(parts[2]))
                gripper_slip_boost_active = bool(int(parts[3]))
                t = float(parts[4])
                values = [float(value) for value in parts[5:]]
            elif len(parts) == 76:
                gripper_contact_hold_active = bool(int(parts[2]))
                gripper_slip_boost_active = False
                t = float(parts[3])
                values = [float(value) for value in parts[4:]]
            else:
                gripper_contact_hold_active = False
                gripper_slip_boost_active = False
                t = float(parts[2])
                values = [float(value) for value in parts[3:]]
        except ValueError:
            return

        triples = [
            (values[i], values[i + 1], values[i + 2])
            for i in range(0, len(values), 3)
        ]
        leader = triples[:8]
        reference = triples[8:16]
        follower = triples[16:24]
        state = self.states[side]
        if state.first_wall is None:
            state.first_wall = time.monotonic()
        if state.leader:
            deltas = [abs(now[0] - prev[0]) for now, prev in zip(leader, state.leader)]
            state.max_abs_delta_q = max(state.max_abs_delta_q, max(deltas, default=0.0))
        state.last_leader = state.leader
        state.leader = leader
        state.reference = reference
        state.follower = follower
        state.live_started = live_started
        state.gripper_contact_hold_active = gripper_contact_hold_active
        state.gripper_slip_boost_active = gripper_slip_boost_active
        state.last_t = t
        state.last_wall = time.monotonic()
        state.packet_count += 1
        state.max_abs_dq = max(state.max_abs_dq, max((abs(item[1]) for item in leader), default=0.0))
        errors = [abs(ref[0] - fb[0]) for ref, fb in zip(reference, follower)]
        state.max_abs_ref_error = max(state.max_abs_ref_error, max(errors, default=0.0))

    def draw(self) -> None:
        now = time.monotonic()
        lines: list[str] = []
        if self.bind_errors:
            lines.append("Some monitor ports are already in use by another monitor window:")
            lines.extend(f"  {error}" for error in self.bind_errors)
            lines.append("Close the existing monitor window before opening another one on the same port.")
            lines.append("")
        for key in ("right_arm", "left_arm"):
            state = self.states[key]
            stale = state.last_wall is None or now - state.last_wall > self.stale_s
            age = (now - state.last_wall) if state.last_wall is not None else float("inf")
            elapsed = (now - state.first_wall) if state.first_wall is not None else 0.0
            hz = state.packet_count / elapsed if elapsed > 1e-6 else 0.0
            ok = "OK" if not stale else "NO DATA"
            if state.gripper_slip_boost_active:
                control_mode = "force-position/slip-boost"
            elif state.gripper_contact_hold_active:
                control_mode = "force-position/contact-hold"
            else:
                control_mode = "position"
            lines.append(
                f"{key}  {ok}  packets={state.packet_count}  rate={hz:6.1f}Hz  "
                f"last_age={age:5.2f}s  live_started={int(state.live_started)}  "
                f"gripper_mode={control_mode}  "
                f"max|leader dq|={state.max_abs_dq:.4f}  maxΔq={state.max_abs_delta_q:.5f}  "
                f"max|ref-feedback_q|={state.max_abs_ref_error:.5f}"
            )
            lines.append(
                "  joint      leader_q    leader_dq  leader_tau       ref_q      ref_dq  "
                "   ref_tau  feedback_q feedback_dq feedback_tau  ref-feedback_q"
            )
            if state.leader and state.reference and state.follower:
                for idx, name in enumerate(JOINT_NAMES):
                    lq, ldq, ltau = state.leader[idx]
                    rq, rdq, rtau = state.reference[idx]
                    fq, fdq, ftau = state.follower[idx]
                    err = rq - fq
                    lines.append(
                        f"  {name:<5} {lq:11.5f} {ldq:11.5f} {ltau:11.5f} "
                        f"{rq:11.5f} {rdq:11.5f} {rtau:10.5f} "
                        f"{fq:11.5f} {fdq:11.5f} {ftau:12.5f} {err:15.5f}"
                    )
            else:
                lines.append("  waiting for packets...")
            lines.append("")

        self.text.delete("1.0", tk.END)
        self.text.insert("1.0", "\n".join(lines))


def main() -> int:
    parser = argparse.ArgumentParser(description="OpenArm real teleop leader monitor panel.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--right-port", type=int, default=47950)
    parser.add_argument("--left-port", type=int, default=47951)
    parser.add_argument("--stale", type=float, default=0.5)
    args = parser.parse_args()
    TeleopMonitorPanel(args.host, args.right_port, args.left_port, args.stale).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
