"""Live motor MOS/rotor temperature monitor for dual_replay during run_once.

Tails the motor torque+temp CSV written by dual_replay_control and:
  - prints a compact table every few seconds
  - appends the same lines to motor_temps_live.log in the run directory
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import TextIO

CHANNEL_NAMES = [
    *(f"L-J{i}" for i in range(1, 8)),
    "L-Grip",
    *(f"R-J{i}" for i in range(1, 8)),
    "R-Grip",
]


class MotorTempMonitor:
    def __init__(
        self,
        csv_path: Path,
        *,
        live_log: Path | None = None,
        print_period_s: float = 2.0,
        warn_rotor_c: float = 70.0,
    ) -> None:
        self.csv_path = Path(csv_path)
        self.live_log = Path(live_log) if live_log is not None else self.csv_path.with_name(
            "motor_temps_live.log"
        )
        self.print_period_s = max(0.5, float(print_period_s))
        self.warn_rotor_c = float(warn_rotor_c)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_print = 0.0
        self._max_rotor = [0] * 16
        self._max_mos = [0] * 16

    def start(self) -> None:
        if self._thread is not None:
            return
        self.live_log.parent.mkdir(parents=True, exist_ok=True)
        with self.live_log.open("a", encoding="utf-8") as fh:
            fh.write(
                f"# motor temp monitor start csv={self.csv_path} period={self.print_period_s}s\n"
            )
        self._thread = threading.Thread(target=self._run, name="motor-temp-monitor", daemon=True)
        self._thread.start()
        print(
            f"[motor-temp] live monitor ON -> csv={self.csv_path} log={self.live_log} "
            f"(print every {self.print_period_s:.1f}s)",
            flush=True,
        )

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._print_peak_summary()

    def _print_peak_summary(self) -> None:
        hot_i = max(range(16), key=lambda i: self._max_rotor[i])
        line = (
            "[motor-temp] session peak rotor: "
            + " ".join(f"{CHANNEL_NAMES[i]}={self._max_rotor[i]}C" for i in range(16))
            + f" | hottest {CHANNEL_NAMES[hot_i]}={self._max_rotor[hot_i]}C"
        )
        print(line, flush=True)
        try:
            with self.live_log.open("a", encoding="utf-8") as fh:
                fh.write(line + "\n")
        except OSError:
            pass

    def _run(self) -> None:
        # Wait for CSV to appear (server startup).
        deadline = time.monotonic() + 60.0
        while not self._stop.is_set() and not self.csv_path.is_file():
            if time.monotonic() >= deadline:
                print(f"[motor-temp][WARN] CSV not found: {self.csv_path}", flush=True)
                return
            time.sleep(0.2)

        try:
            with self.csv_path.open("r", encoding="utf-8", errors="ignore") as fh:
                # Skip header.
                header = fh.readline()
                if not header:
                    pass
                while not self._stop.is_set():
                    line = fh.readline()
                    if not line:
                        time.sleep(0.05)
                        continue
                    self._handle_row(line)
        except OSError as exc:
            print(f"[motor-temp][WARN] monitor stopped: {exc}", flush=True)

    def _handle_row(self, line: str) -> None:
        parts = line.strip().split(",")
        # time,cycle + 16 tau + 16 mos + 16 rotor + 16 err + 16 q
        if len(parts) < 2 + 16 * 4:
            return
        try:
            t = float(parts[0])
            mos = [int(float(parts[2 + 16 + i])) for i in range(16)]
            rotor = [int(float(parts[2 + 32 + i])) for i in range(16)]
            tau = [float(parts[2 + i]) for i in range(16)]
        except (TypeError, ValueError):
            return
        for i in range(16):
            self._max_mos[i] = max(self._max_mos[i], mos[i])
            self._max_rotor[i] = max(self._max_rotor[i], rotor[i])

        now = time.monotonic()
        if now - self._last_print < self.print_period_s:
            return
        self._last_print = now
        self._emit(t, mos, rotor, tau)

    def _emit(self, t: float, mos: list[int], rotor: list[int], tau: list[float]) -> None:
        def fmt8(vals: list[int], off: int) -> str:
            return " ".join(f"{vals[off + i]:3d}" for i in range(8))

        hot_i = max(range(16), key=lambda i: rotor[i])
        warn = " WARN" if rotor[hot_i] >= self.warn_rotor_c else ""
        line = (
            f"[motor-temp] t={t:7.1f}s "
            f"L_mos=[{fmt8(mos, 0)}] L_rot=[{fmt8(rotor, 0)}] | "
            f"R_mos=[{fmt8(mos, 8)}] R_rot=[{fmt8(rotor, 8)}] | "
            f"hot {CHANNEL_NAMES[hot_i]} rot={rotor[hot_i]}C mos={mos[hot_i]}C "
            f"tau={tau[hot_i]:+.2f}Nm{warn}"
        )
        print(line, flush=True)
        try:
            with self.live_log.open("a", encoding="utf-8") as fh:
                fh.write(line + "\n")
        except OSError:
            pass


def configure_motor_log_for_run(run_dir: Path, *, hz: float | None = None) -> Path:
    """Point dual_replay motor CSV into this run directory (via env)."""
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    csv_path = run_dir / "motor_torque_temp.csv"
    os.environ["OPENARM_REPLAY_MOTOR_LOG"] = str(csv_path)
    if hz is not None:
        os.environ["OPENARM_REPLAY_MOTOR_LOG_HZ"] = str(hz)
    else:
        os.environ["OPENARM_REPLAY_MOTOR_LOG_HZ"] = "100"
    # Realtime console from C++ server as well.
    os.environ["OPENARM_REPLAY_TEMP_PRINT"] = os.environ.get("OPENARM_REPLAY_TEMP_PRINT", "1")
    os.environ["OPENARM_REPLAY_TEMP_PRINT_S"] = os.environ.get("OPENARM_REPLAY_TEMP_PRINT_S", "2")
    return csv_path


def start_monitor_for_run(run_dir: Path) -> MotorTempMonitor:
    csv_path = configure_motor_log_for_run(run_dir)
    period = float(os.environ.get("OPENARM_REPLAY_TEMP_PRINT_S", "2") or "2")
    mon = MotorTempMonitor(
        csv_path,
        live_log=run_dir / "motor_temps_live.log",
        print_period_s=period,
    )
    mon.start()
    return mon
