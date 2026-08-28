#!/usr/bin/env python3
"""Persistent Tk panel for commanding real dual-arm replay."""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, simpledialog


def safe_name(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in name) or "action"


class RealReplaySelectPanel:
    def __init__(self, project_dir: Path) -> None:
        self.project_dir = project_dir
        self.server_script = project_dir / "script" / "run_cpp_real_replay_server.sh"
        self.prepare_script = project_dir / "script" / "prepare_dual_replay_directory.py"
        self.torque_panel_script = project_dir / "script" / "torque_telemetry_panel.py"
        self.face_script = project_dir.parent / "face-emoji" / "app" / "face.py"
        self.command_port = int(os.environ.get("OPENARM_REPLAY_COMMAND_PORT", "47970"))
        self.status_port = int(os.environ.get("OPENARM_REPLAY_STATUS_PORT", "47971"))
        self.next_torque_port = int(os.environ.get("OPENARM_TORQUE_PANEL_BASE_PORT", "47980"))
        self.face_enabled = os.environ.get("OPENARM_FACE_EMOJI_ENABLED", "1") != "0"
        self.face_host = os.environ.get("OPENARM_FACE_EMOJI_HOST", "192.168.2.199")
        self.face_port = int(os.environ.get("OPENARM_FACE_EMOJI_PORT", "22345"))
        self.face_timeout = float(os.environ.get("OPENARM_FACE_EMOJI_TIMEOUT", "5.0"))
        self.server_process: subprocess.Popen[str] | None = None
        self.server_ready = False
        self.server_busy = False
        self.pending_torque_panels: dict[int, tuple[str, bool]] = {}
        self.loop_enabled = False
        self.loop_path: Path | None = None
        self.loop_label = ""
        self.loop_side = ""
        self.loop_port = 0
        self.loop_speed = 1.0
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.status_socket.bind(("127.0.0.1", self.status_port))
        self.status_socket.setblocking(False)
        self.sudo_password: str | None = os.environ.get("OPENARM_SUDO_PASSWORD")
        self.closing = False
        self.close_requested = False

        self.root = tk.Tk()
        self.root.title("OpenArm Persistent Real Replay")
        self.root.geometry("1040x680")

        self.status = tk.StringVar(value="Preparing dual-arm replay server...")
        tk.Label(self.root, textvariable=self.status, anchor="w").pack(fill=tk.X, padx=8, pady=4)

        controls = tk.Frame(self.root)
        controls.pack(fill=tk.X, padx=8, pady=4)
        self.speed_var = tk.StringVar(value=os.environ.get("OPENARM_REPLAY_PANEL_SPEED", "1.0"))
        tk.Label(controls, text="速度:").pack(side=tk.LEFT, padx=(0, 2))
        self.speed_spinbox = tk.Spinbox(
            controls,
            from_=0.1,
            to=5.0,
            increment=0.1,
            width=5,
            textvariable=self.speed_var,
        )
        self.speed_spinbox.pack(side=tk.LEFT, padx=(0, 8))
        tk.Button(controls, text="选择文件夹并执行", command=self.select_folder).pack(side=tk.LEFT, padx=4)
        tk.Button(controls, text="选择单个轨迹文件并执行", command=self.select_file).pack(side=tk.LEFT, padx=4)
        tk.Button(controls, text="选择 CSV 循环播放", command=self.select_loop_file).pack(side=tk.LEFT, padx=4)
        tk.Button(controls, text="选择文件夹循环播放", command=self.select_loop_folder).pack(side=tk.LEFT, padx=4)
        tk.Button(controls, text="停止循环播放", command=self.stop_loop).pack(side=tk.LEFT, padx=4)
        tk.Button(controls, text="重启双臂常驻线程", command=self.restart_server).pack(side=tk.LEFT, padx=4)
        tk.Button(controls, text="退出面板并停止线程", command=self.close).pack(side=tk.LEFT, padx=4)

        hint = (
            "面板启动后默认进入 dual_replay_control 双臂常驻服务。\n"
            "双臂和单臂 CSV 都统一由双臂服务执行；单臂 CSV 会补齐另一臂静止轨迹。\n"
            "文件夹按钮可选择任意直接包含 16 关节 CSV 的目录，也支持标准 session/sessions 目录；循环按钮用于疲劳测试。"
        )
        tk.Label(self.root, text=hint, anchor="w", justify=tk.LEFT).pack(fill=tk.X, padx=8, pady=4)

        self.output = tk.Text(self.root, font=("TkFixedFont", 10), wrap="none")
        self.output.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        import atexit

        atexit.register(self._atexit_kill_server)
        # Must stay in __init__: schedule server start + status poll after the window exists.
        self.root_after(100, self.start_server)
        self.root_after(100, self.poll_status_messages)

    def _atexit_kill_server(self) -> None:
        """If the panel process dies abruptly, still try to stop dual_replay."""
        process = self.server_process
        if process is None or process.poll() is not None:
            return
        try:
            self.command_socket.sendto(b"shutdown", ("127.0.0.1", self.command_port))
        except OSError:
            pass
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except (OSError, ProcessLookupError):
            try:
                process.terminate()
            except OSError:
                pass
        try:
            process.wait(timeout=3)
        except Exception:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except (OSError, ProcessLookupError):
                pass

    def run(self) -> None:
        self.root.mainloop()

    def root_after(self, delay_ms: int, callback, *args) -> None:
        if self.closing:
            return
        try:
            self.root.after(delay_ms, callback, *args)
        except tk.TclError:
            pass

    def start_server(self) -> None:
        if self.server_process is not None and self.server_process.poll() is None:
            return
        self.server_ready = False
        if not self.server_script.exists():
            messagebox.showerror("OpenArm Replay", f"server 脚本不存在: {self.server_script}")
            return
        env = os.environ.copy()
        env.setdefault("OPENARM_DUAL_REPLAY_INITIAL_TRANSITION_DURATION", "3.0")
        env.setdefault("OPENARM_REPLAY_COMMAND_PORT", str(self.command_port))
        env.setdefault("OPENARM_REPLAY_STATUS_PORT", str(self.status_port))
        if env.get("OPENARM_AUTO_CONFIG_CAN", "1") == "1":
            password = self.get_sudo_password_if_needed()
            if password is None:
                self.status.set("Replay server start cancelled: sudo password is required for CAN config.")
                return
            if password:
                env["OPENARM_SUDO_PASSWORD"] = password
        self.status.set(f"Starting persistent replay server on UDP {self.command_port}...")
        self.server_process = subprocess.Popen(
            ["bash", str(self.server_script)],
            cwd=str(self.project_dir),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        threading.Thread(target=self.read_server_output, daemon=True).start()

    def get_sudo_password_if_needed(self) -> str | None:
        if self.sudo_password:
            return self.sudo_password
        try:
            result = subprocess.run(
                ["sudo", "-n", "true"],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
                timeout=2.0,
            )
        except subprocess.TimeoutExpired:
            result = subprocess.CompletedProcess(["sudo", "-n", "true"], returncode=1)
        if result.returncode == 0:
            return ""
        try:
            self.root.deiconify()
            self.root.lift()
            self.root.focus_force()
            self.root.attributes("-topmost", True)
            self.root.after(500, lambda: self.root.attributes("-topmost", False))
        except tk.TclError:
            pass
        password = simpledialog.askstring(
            "OpenArm sudo password",
            "动作回放启动时需要 sudo 重新配置 CAN，请输入当前用户密码：",
            show="*",
            parent=self.root,
        )
        if password:
            self.sudo_password = password
        return password

    def restart_server(self) -> None:
        self.stop_loop()
        self.stop_server()
        self.output.delete("1.0", tk.END)
        self.start_server()

    def read_server_output(self) -> None:
        process = self.server_process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            self.root_after(0, self.append_output, line)
            if "ready on UDP port" in line:
                self.server_ready = True
                self.root_after(0, self.status.set, "Replay server ready. 请选择动作轨迹。")
        exit_code = process.wait()
        self.server_ready = False
        self.root_after(0, self.status.set, f"Replay server exited with code {exit_code}")

    def append_output(self, line: str) -> None:
        self.output.insert(tk.END, line)
        self.output.see(tk.END)

    def poll_status_messages(self) -> None:
        while True:
            try:
                packet, _ = self.status_socket.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                return
            text = packet.decode("utf-8", errors="replace").strip()
            self.handle_status_message(text)
        self.root_after(100, self.poll_status_messages)

    def handle_status_message(self, text: str) -> None:
        if not text:
            return
        parts = text.split("|", 1)
        state = parts[0]
        detail = parts[1] if len(parts) > 1 else ""
        status_map = {
            "ready": "Replay server ready. 请选择动作轨迹。",
            "accepted": f"Replay accepted: {detail}",
            "started": f"Replay started: {detail}",
            "finished": "Replay finished; holding final pose.",
            "busy": f"Replay server busy, ignored: {detail}",
            "load_failed": f"Replay load failed: {detail}",
            "transition_added": f"Initial transition added: {detail}",
            "transition_skipped": f"Initial transition skipped: {detail}",
            "transition_failed": f"Initial transition failed: {detail}",
            "safety": f"Safety triggered: {detail}",
            "action_started": f"Action started: {detail}",
            "shutdown_received": "Replay server is switching to damping release...",
            "shutdown_done": "Replay server sent damping release and stopped.",
        }
        if state == "ready":
            self.server_ready = True
        if state in {"accepted", "started"}:
            self.server_busy = True
        elif state in {"ready", "finished", "load_failed", "safety", "shutdown_done"}:
            self.server_busy = False
        self.status.set(status_map.get(state, text))
        self.append_output(f"[status] {text}\n")
        if state == "accepted":
            self.open_pending_torque_panel(detail)
        if state == "action_started":
            self.handle_action_started(detail)
        if state == "finished" and self.loop_enabled:
            self.root_after(200, self.continue_loop_play)
        elif state in {"load_failed", "safety", "shutdown_done"}:
            self.loop_enabled = False

    def handle_action_started(self, detail: str) -> None:
        fields = self.parse_status_fields(detail)
        label = fields.get("label", "action")
        face_emoji = fields.get("face_emoji", "").strip()
        if not face_emoji:
            self.append_output(f"[face-emoji] {label}: no face_emoji metadata, skip\n")
            return
        self.send_face_emoji_async(label, face_emoji)

    @staticmethod
    def parse_status_fields(detail: str) -> dict[str, str]:
        fields: dict[str, str] = {}
        for part in detail.split("|"):
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            fields[key.strip()] = value.strip()
        return fields

    def parse_face_emoji_spec(self, spec: str) -> dict[str, str]:
        spec = spec.strip()
        if not spec:
            raise ValueError("empty face_emoji spec")
        if spec.startswith("{"):
            data = json.loads(spec)
            msg_type = str(data.get("type", "expression"))
            intent = str(data.get("intent", "")).strip()
            tts = data.get("tts")
            if not intent:
                raise ValueError("face_emoji JSON missing intent")
            result = {"type": msg_type, "intent": intent}
            if tts:
                result["tts"] = str(tts)
            return result
        parts = spec.split(":", 2)
        if len(parts) >= 2 and parts[0] in {"expression", "scene"}:
            result = {"type": parts[0], "intent": parts[1]}
            if len(parts) == 3 and parts[2]:
                result["tts"] = parts[2]
            return result
        return {"type": "expression", "intent": spec}

    def send_face_emoji_async(self, label: str, spec: str) -> None:
        if not self.face_enabled:
            self.append_output(f"[face-emoji] disabled; skip {label}: {spec}\n")
            return
        if not self.face_script.exists():
            self.append_output(f"[face-emoji][WARN] script not found: {self.face_script}\n")
            return
        try:
            payload = self.parse_face_emoji_spec(spec)
        except (ValueError, json.JSONDecodeError) as exc:
            self.append_output(f"[face-emoji][WARN] invalid metadata for {label}: {exc}\n")
            return
        thread = threading.Thread(
            target=self.send_face_emoji_worker,
            args=(label, payload),
            daemon=True,
        )
        thread.start()

    def send_face_emoji_worker(self, label: str, payload: dict[str, str]) -> None:
        cmd = [
            sys.executable,
            str(self.face_script),
            "--host",
            self.face_host,
            "--port",
            str(self.face_port),
            "--timeout",
            f"{self.face_timeout:.3f}",
            "--type",
            payload["type"],
            "--intent",
            payload["intent"],
        ]
        if payload.get("tts"):
            cmd += ["--tts", payload["tts"]]
        try:
            result = subprocess.run(
                cmd,
                cwd=str(self.face_script.parent.parent),
                text=True,
                capture_output=True,
                timeout=self.face_timeout + 1.0,
            )
        except subprocess.TimeoutExpired:
            self.root_after(
                0,
                self.append_output,
                f"[face-emoji][WARN] timeout for {label}: {payload}\n",
            )
            return
        output = (result.stdout or result.stderr or "").strip()
        if result.returncode == 0:
            self.root_after(
                0,
                self.append_output,
                f"[face-emoji] sent for {label}: {payload}"
                + (f" ({output})\n" if output else "\n"),
            )
        else:
            self.root_after(
                0,
                self.append_output,
                f"[face-emoji][WARN] failed for {label}: rc={result.returncode} {output}\n",
            )

    def select_folder(self) -> None:
        path = filedialog.askdirectory(
            title="选择任意包含 merged CSV 的文件夹，或 session/sessions 文件夹",
            initialdir=str(self.project_dir / "recordings" / "sessions"),
        )
        if not path:
            return
        try:
            replay_path = self.prepare_folder(Path(path))
        except Exception as exc:
            messagebox.showerror("OpenArm Replay", str(exc))
            return
        self.send_play_command(replay_path, Path(path).name, "")

    def select_file(self) -> None:
        path_text = filedialog.askopenfilename(
            title="选择单个动作轨迹 CSV",
            initialdir=str(self.project_dir / "recordings" / "sessions"),
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if not path_text:
            return
        path = Path(path_text)
        side = self.infer_single_arm_side(path)
        if side == "ambiguous":
            messagebox.showwarning(
                "OpenArm Replay",
                "该 CSV 看起来是单臂 8 关节轨迹，但无法从路径判断 left/right；"
                "请把文件放在 left/right 目录下，或在文件名中包含 left/right。",
            )
            return
        self.send_play_command(path, path.stem, side)

    def select_loop_file(self) -> None:
        if self.server_busy:
            messagebox.showwarning("OpenArm Replay", "当前动作还在执行，请等待 finished 状态后再启动循环播放。")
            return
        path_text = filedialog.askopenfilename(
            title="选择要循环播放的 CSV",
            initialdir=str(self.project_dir / "recordings" / "sessions"),
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if not path_text:
            return
        path = Path(path_text)
        side = self.infer_single_arm_side(path)
        if side == "ambiguous":
            messagebox.showwarning(
                "OpenArm Replay",
                "该 CSV 看起来是单臂 8 关节轨迹，但无法从路径判断 left/right；"
                "请把文件放在 left/right 目录下，或在文件名中包含 left/right。",
            )
            return
        self.loop_enabled = True
        self.loop_path = path
        self.loop_label = path.stem
        self.loop_side = side
        self.loop_port = self.next_torque_port
        self.loop_speed = self.current_speed()
        self.next_torque_port += 1
        self.open_torque_panel(f"{self.loop_label}_loop", self.loop_port)
        self.append_output(
            f"[panel] loop replay enabled: path={path.resolve()} side={side or 'dual'}\n"
        )
        self.send_play_command(
            path,
            self.loop_label,
            side,
            telemetry_port=self.loop_port,
            open_panel=False,
        )

    def select_loop_folder(self) -> None:
        if self.server_busy:
            messagebox.showwarning("OpenArm Replay", "当前动作还在执行，请等待 finished 状态后再启动循环播放。")
            return
        path_text = filedialog.askdirectory(
            title="选择要循环播放的文件夹",
            initialdir=str(self.project_dir / "recordings" / "sessions"),
        )
        if not path_text:
            return
        try:
            replay_path = self.prepare_folder(Path(path_text))
        except Exception as exc:
            messagebox.showerror("OpenArm Replay", str(exc))
            return
        self.loop_enabled = True
        self.loop_path = replay_path
        self.loop_label = Path(path_text).name
        self.loop_side = ""
        self.loop_port = self.next_torque_port
        self.loop_speed = self.current_speed()
        self.next_torque_port += 1
        self.open_torque_panel(f"{self.loop_label}_folder_loop", self.loop_port)
        self.append_output(
            f"[panel] folder loop replay enabled: selected={Path(path_text).resolve()} replay_path={replay_path.resolve()}\n"
        )
        self.send_play_command(
            replay_path,
            self.loop_label,
            "",
            telemetry_port=self.loop_port,
            open_panel=False,
        )

    def continue_loop_play(self) -> None:
        if not self.loop_enabled or self.loop_path is None:
            return
        if self.server_busy:
            self.root_after(200, self.continue_loop_play)
            return
        self.append_output(
            f"[panel] loop replay next cycle: path={self.loop_path.resolve()} side={self.loop_side or 'dual'}\n"
        )
        self.send_play_command(
            self.loop_path,
            self.loop_label,
            self.loop_side,
            telemetry_port=self.loop_port,
            open_panel=False,
            quiet_busy=True,
            speed=self.loop_speed,
        )

    def stop_loop(self) -> None:
        self.loop_enabled = False
        self.loop_path = None
        self.loop_label = ""
        self.loop_side = ""
        self.loop_port = 0
        self.loop_speed = 1.0
        self.status.set("Loop replay stopped. 当前动作若已开始会执行到本次结束。")
        self.append_output("[panel] loop replay disabled\n")

    def prepare_folder(self, path: Path) -> Path:
        if self.folder_has_direct_merged_csv(path):
            self.append_output(f"[panel] using selected CSV folder directly: {path}\n")
            return path
        if any(path.glob("*.csv")):
            raise RuntimeError(
                f"文件夹中存在 CSV，但没有检测到 16 关节 merged CSV: {path}\n"
                "请确认 CSV 表头为 time,q0,dq0[,tau0],...,q15,dq15[,tau15]，或选择标准 session/sessions 文件夹。"
            )

        result = subprocess.run(
            [sys.executable, str(self.prepare_script), str(path)],
            cwd=str(self.project_dir),
            text=True,
            capture_output=True,
            check=False,
        )
        if result.stdout:
            self.append_output(result.stdout)
        if result.stderr:
            self.append_output(result.stderr)
        if result.returncode != 0:
            raise RuntimeError(f"准备目录失败: {path}")

        merged_dir = ""
        for line in result.stdout.splitlines():
            if line.startswith("MERGED_DIR="):
                merged_dir = line.split("=", 1)[1].strip()
                break
        if not merged_dir:
            raise RuntimeError("该文件夹没有可直接执行的 merged 轨迹；请选择包含 16 关节 CSV 的任意文件夹、标准 session 文件夹、merged CSV 或单臂 CSV。")
        self.append_output(f"[panel] prepared selected session folder into: {merged_dir}\n")
        return Path(merged_dir)

    def folder_has_direct_merged_csv(self, path: Path) -> bool:
        if not path.is_dir():
            return False
        for csv_path in sorted(path.glob("*.csv")):
            if self.csv_joint_layout(csv_path) == "merged":
                return True
        return False

    def csv_joint_layout(self, path: Path) -> str:
        try:
            with path.open(errors="replace") as file:
                header = file.readline().strip().split(",")
        except OSError:
            return "unknown"
        if len(header) >= 49 or (len(header) >= 33 and "q15" in header and "dq15" in header):
            return "merged"
        if len(header) >= 25 or (len(header) >= 17 and "q7" in header and "dq7" in header):
            return "single"
        return "unknown"

    def infer_single_arm_side(self, path: Path) -> str:
        layout = self.csv_joint_layout(path)
        if layout == "merged":
            return ""
        lowered = str(path).lower()
        if "left" in lowered:
            return "left"
        if "right" in lowered:
            return "right"
        return "ambiguous" if layout == "single" else ""

    def send_play_command(
        self,
        replay_path: Path,
        label: str,
        side: str,
        telemetry_port: int | None = None,
        open_panel: bool = True,
        quiet_busy: bool = False,
        speed: float | None = None,
    ) -> None:
        if self.server_busy:
            if not quiet_busy:
                messagebox.showwarning("OpenArm Replay", "当前动作还在执行，请等待 finished 状态后再选择新轨迹。")
            return
        if self.server_process is None or self.server_process.poll() is not None:
            self.start_server()
        if not self.server_ready:
            messagebox.showwarning("OpenArm Replay", "回放服务还没有 ready，请等状态显示 ready 后再选择轨迹。")
            return
        if telemetry_port is None:
            port = self.next_torque_port
            self.next_torque_port += 1
        else:
            port = telemetry_port
        title = label or replay_path.stem
        if open_panel:
            self.pending_torque_panels[port] = (title, True)
        elif telemetry_port is not None:
            self.pending_torque_panels.setdefault(port, (title, False))
        replay_speed = speed if speed is not None else self.current_speed()
        message = f"play|{replay_path.resolve()}|{port}|{safe_name(title)}|{side}|{replay_speed:.3f}"
        self.append_output(
            f"[panel] sending replay command: path={replay_path.resolve()} "
            f"side={side or 'dual'} speed={replay_speed:.3f}x\n"
        )
        self.command_socket.sendto(message.encode("utf-8"), ("127.0.0.1", self.command_port))
        side_text = f" ({side} arm only)" if side else ""
        self.status.set(f"Sent replay command: {title}{side_text}, speed={replay_speed:.3f}x")

    def open_pending_torque_panel(self, detail: str) -> None:
        fields = self.parse_status_fields(detail)
        port_text = fields.get("telemetry_port", "")
        if not port_text:
            return
        try:
            port = int(port_text)
        except ValueError:
            return
        pending = self.pending_torque_panels.pop(port, None)
        if pending is None:
            return
        title, should_open = pending
        if should_open:
            self.open_torque_panel(title, port)

    def current_speed(self) -> float:
        try:
            speed = float(self.speed_var.get())
        except ValueError:
            speed = 1.0
        speed = max(0.1, min(speed, 5.0))
        self.speed_var.set(f"{speed:.1f}")
        return speed

    def open_torque_panel(self, title: str, port: int) -> None:
        output_dir = self.project_dir / "recordings" / "torque_telemetry"
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        output_path = output_dir / f"{stamp}_{safe_name(title)}_torque.csv"
        subprocess.Popen(
            [
                sys.executable,
                str(self.torque_panel_script),
                "--port",
                str(port),
                "--output",
                str(output_path),
                "--title",
                f"OpenArm Torque - {title}",
            ],
            cwd=str(self.project_dir),
            start_new_session=True,
        )

    def stop_server(
        self,
        block: bool = True,
        timeout_s: float | None = None,
        force_after_timeout: bool = True,
        on_exit=None,
    ) -> None:
        process = self.server_process
        if process is None or process.poll() is not None:
            if on_exit is not None:
                on_exit()
            return
        self.status.set("Sending damping release and stopping replay server...")
        try:
            self.command_socket.sendto(b"shutdown", ("127.0.0.1", self.command_port))
        except OSError:
            pass

        if timeout_s is None and force_after_timeout:
            timeout_s = float(os.environ.get("OPENARM_REPLAY_SHUTDOWN_WAIT", "60"))
        if block:
            self.wait_for_server_exit(process, timeout_s, force_after_timeout, on_exit)
        else:
            threading.Thread(
                target=self.wait_for_server_exit,
                args=(process, timeout_s, force_after_timeout, on_exit),
                daemon=True,
            ).start()

    def wait_for_server_exit(
        self,
        process: subprocess.Popen[str],
        timeout_s: float | None,
        force_after_timeout: bool,
        on_exit=None,
    ) -> None:
        if timeout_s is None:
            process.wait()
        else:
            try:
                process.wait(timeout=max(0.0, timeout_s))
            except subprocess.TimeoutExpired:
                pass
        if process.poll() is None:
            if not force_after_timeout:
                self.root_after(
                    0,
                    self.status.set,
                    "Waiting for replay server to finish safe damping release...",
                )
                return
            self.root_after(0, self.status.set, "Graceful shutdown timed out; sending SIGTERM.")
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.root_after(0, self.status.set, "SIGTERM timed out; forcing replay server to stop.")
                os.killpg(process.pid, signal.SIGKILL)
        if on_exit is not None:
            self.root_after(0, on_exit)

    def close(self) -> None:
        if self.close_requested:
            return
        self.close_requested = True
        self.closing = True
        self.stop_loop()
        self.closing = False
        # Send graceful shutdown (Kd-only settle), then always tear down the process group.
        damp_sec = float(os.environ.get("OPENARM_REPLAY_DAMP_SEC", "6"))
        wait_sec = float(os.environ.get("OPENARM_REPLAY_SHUTDOWN_WAIT", str(damp_sec + 8.0)))
        self.status.set(
            f"Closing: Kd damping settle (~{damp_sec:.0f}s) then exit dual_replay_control..."
        )
        self.stop_server(
            block=False,
            timeout_s=wait_sec,
            force_after_timeout=True,
            on_exit=self.finish_close,
        )

    def finish_close(self) -> None:
        self.closing = True
        try:
            self.command_socket.close()
        except OSError:
            pass
        try:
            self.status_socket.close()
        except OSError:
            pass
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description="OpenArm real replay selection panel.")
    parser.add_argument(
        "--project-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="openarm_inplace project directory",
    )
    args = parser.parse_args()
    RealReplaySelectPanel(args.project_dir.resolve()).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
