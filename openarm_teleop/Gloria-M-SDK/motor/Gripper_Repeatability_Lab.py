# -*- coding: utf-8 -*-
"""
Gripper Repeatability Lab —— 达妙 DM4310 夹爪行程重复定位精度测试台 (GUI)
==========================================================================

定位（与其它脚本的区别）
--------------------------------------------------------------------------
- Gripper_Studio.py          : 通用控制 + 多曲线可视化 + 保护调试。
- Motor_Param_Tool.py        : 电机寄存器参数离线配置。
- Collision_Impedance_Lab.py : 碰撞/阻抗/零位 专项测试。
- 本脚本                     : 专测【行程重复定位精度（repeatability）】——在两个
                               行程端点 A/B 之间往返 N 次，记录每次实际到位位置，
                               统计离散度（σ / 极差 / 重复精度±），带实时轨迹与散点图。

测试原理
--------------------------------------------------------------------------
往返一个循环 = 去 A → 停留settle → 记录A到位位置 → 去 B → 停留 → 记录B到位位置。
重复 N 次后，对 A、B 各得 N 个到位样本：
  - 均值 mean、样本标准差 σ、极差(max-min)
  - 重复定位精度 ≈ ±极差/2 （也给出 ±3σ 供参考）
“到位”判定：实际位置接近目标(容差 tol) 且 速度低于阈值(已稳定)。

可调参数：速度、刚性(Kp)、阻尼(Kd)、加持力(合拢端夹持力矩)、行程端点 A/B、
          重复次数、到位停留时间、到位容差。

架构：串口控制环跑在后台 RepeatWorker 线程(200Hz 位置循环状态机)，GUI 主线程
只读共享状态并刷新曲线。所有 Tk 调用只在主线程，串口只在 worker 线程。

运行 (motor/ 目录下)：  python Gripper_Repeatability_Lab.py
依赖：pyserial, numpy, matplotlib (TkAgg), tkinter
"""
import time
import csv
import queue
import statistics
import threading
import collections

import serial
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from CAN import (Motor, MotorControl, DM_Motor_Type, DM_variable)
from XuanYa_Comm import MotorComm

# ----------------- 常量 -----------------
DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 921600
SEND_HZ = 200.0
SEND_DT = 1.0 / SEND_HZ
WINDOW_S = 16.0
BUF_LEN = int(SEND_HZ * (WINDOW_S + 4))
REFRESH_MS = 80

# 行程端点默认值 (来自夹爪实测标定: 全开≈-0.4, 全合≈-2.6)
DEF_POS_A = -0.40   # 端点 A (张开侧)
DEF_POS_B = -2.60   # 端点 B (合拢侧)

# 状态机相位
PH_IDLE = "空闲"
PH_TO_A = "前往A"
PH_DWELL_A = "A停留"
PH_TO_B = "前往B"
PH_DWELL_B = "B停留"
PH_DONE = "完成"


class RepeatWorker(threading.Thread):
    """后台控制线程: 独占串口, 跑 200Hz 位置循环状态机, 采集到位样本。"""

    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.cmd_q = queue.Queue()
        self.buf = collections.deque(maxlen=BUF_LEN)
        self.stop_flag = False

        self.serial = None
        self.mc = None
        self.motor = None
        self.comm = MotorComm()
        self.connected = False
        self.enabled = False

        # 共享可调参数 (GUI 写)
        self.p = dict(
            pos_a=DEF_POS_A, pos_b=DEF_POS_B,
            speed=2.0, kp=12.0, kd=1.0, grip_force=0.0,
            dwell=0.6, tol=0.01, repeats=20,
            vel_settle=0.06, move_timeout=10.0,
            prot_temp_on=True, temp_limit=80.0,
        )

        # 测试运行态
        self.test_running = False
        self.phase = PH_IDLE
        self.cycle = 0
        self.cfg = {}            # 测试开始时冻结的参数快照
        self.samples_a = []      # [(cycle, pos)]
        self.samples_b = []
        self.phase_start = 0.0
        self.dwell_start = 0.0

        self.q_slew = 0.0
        self.pmax = 3.14
        self.state = "未连接"
        self.reason = ""
        self.t0 = time.time()
        self._last = dict(q=0.0, dq=0.0, tau=0.0, tmos=0.0, troto=0.0)

    # ---------- GUI 接口 ----------
    def post(self, *cmd):
        self.cmd_q.put(cmd)

    def set_param(self, key, val):
        with self.lock:
            self.p[key] = val

    def get_params(self):
        with self.lock:
            return dict(self.p)

    def snapshot(self):
        with self.lock:
            return list(self.buf)

    def status(self):
        with self.lock:
            return dict(connected=self.connected, enabled=self.enabled,
                        running=self.test_running, phase=self.phase,
                        cycle=self.cycle, repeats=self.cfg.get("repeats", self.p["repeats"]),
                        state=self.state, reason=self.reason,
                        pmax=self.pmax, **self._last)

    def get_results(self):
        with self.lock:
            return dict(samples_a=list(self.samples_a), samples_b=list(self.samples_b),
                        pos_a=self.cfg.get("pos_a", self.p["pos_a"]),
                        pos_b=self.cfg.get("pos_b", self.p["pos_b"]))

    # ---------- 主循环 ----------
    def run(self):
        while not self.stop_flag:
            try:
                self._process_cmds()
                if self.connected and self.enabled:
                    if self.test_running:
                        self._test_step()
                    else:
                        self._hold_step()
                else:
                    time.sleep(0.005)
            except Exception as e:  # noqa: BLE001
                self.reason = f"loop err: {e}"
                time.sleep(0.02)
        self._safe_shutdown()

    def _process_cmds(self):
        while True:
            try:
                cmd = self.cmd_q.get_nowait()
            except queue.Empty:
                break
            name = cmd[0]
            try:
                if name == "connect":
                    self._connect(cmd[1], cmd[2])
                elif name == "disconnect":
                    self._disconnect()
                elif name == "enable":
                    self._enable()
                elif name == "disable":
                    self._disable()
                elif name == "test_start":
                    self._test_start()
                elif name == "test_stop":
                    self.test_running = False
                    self.phase = PH_IDLE
                    self.state = "已停止"
                elif name == "test_reset":
                    with self.lock:
                        self.samples_a.clear()
                        self.samples_b.clear()
                    self.cycle = 0
                    self.phase = PH_IDLE
                    self.state = "已清空"
            except Exception as e:  # noqa: BLE001
                self.reason = f"cmd {name} err: {e}"

    def _connect(self, port, baud):
        if self.connected:
            return
        self.serial = serial.Serial(port, baud, timeout=0.5)
        self.motor = Motor(DM_Motor_Type.DM4310, 0x01, 0x101)
        self.mc = MotorControl(self.serial)
        self.mc.addMotor(self.motor)
        self._sync_limits()
        self.connected = True
        self.state = "已连接"
        self.t0 = time.time()

    def _sync_limits(self):
        pmax = self.mc.read_motor_param(self.motor, DM_variable.PMAX)
        vmax = self.mc.read_motor_param(self.motor, DM_variable.VMAX)
        tmax = self.mc.read_motor_param(self.motor, DM_variable.TMAX)
        if pmax and vmax and tmax:
            self.mc.change_limit_param(DM_Motor_Type.DM4310, pmax, vmax, tmax)
            with self.lock:
                self.pmax = float(pmax)

    def _disconnect(self):
        self.test_running = False
        self._disable()
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass
        self.connected = False
        self.state = "未连接"

    def _enable(self):
        if not self.connected:
            return
        self.mc.disable(self.motor)
        time.sleep(0.3)
        self.mc.enable(self.motor)
        time.sleep(0.1)
        self.mc.refresh_motor_status(self.motor)
        self.q_slew = self.comm.getPosRad(self.motor)
        self.enabled = True
        self.state = "已使能"
        self.reason = ""

    def _disable(self):
        self.test_running = False
        if self.connected and self.mc:
            for _ in range(10):
                self.mc.controlMIT(self.motor, 0.0, 0.8, 0.0, 0.0, 0.0)
                time.sleep(SEND_DT)
            self.mc.disable(self.motor)
        self.enabled = False
        self.state = "已失能"

    def _test_start(self):
        if not (self.connected and self.enabled):
            self.reason = "测试启动失败: 未连接/未使能"
            return
        self.cfg = self.get_params()
        with self.lock:
            self.samples_a.clear()
            self.samples_b.clear()
        self.cycle = 0
        self.q_slew = self.comm.getPosRad(self.motor)
        self.phase = PH_TO_A
        self.phase_start = time.time()
        self.test_running = True
        self.state = "测试中"
        self.reason = ""

    # ---------- 读温度 + 温度保护 ----------
    def _read_and_protect(self):
        q = self.comm.getPosRad(self.motor)
        dq = self.comm.getVelRad(self.motor)
        tau = self.comm.getTorque(self.motor)
        tmos = float(self.comm.getTMos(self.motor))
        troto = float(self.comm.getTRoto(self.motor))
        self._last = dict(q=q, dq=dq, tau=tau, tmos=tmos, troto=troto)
        p = self.cfg if self.test_running else self.get_params()
        if p["prot_temp_on"] and (tmos >= p["temp_limit"] or troto >= p["temp_limit"]):
            self.mc.controlMIT(self.motor, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.enabled = False
            self.test_running = False
            self.mc.disable(self.motor)
            self.state = "FAULT"
            self.reason = f"过温 {max(tmos, troto):.0f}C >= {p['temp_limit']:.0f}C"
            return None
        return q, dq, tau

    def _hold_step(self):
        """非测试态: 用阻抗在当前位置保持, 防止掉力。"""
        r = self._read_and_protect()
        if r is None:
            return
        p = self.get_params()
        self.mc.controlMIT(self.motor, p["kp"], p["kd"], self.q_slew, 0.0, 0.0)
        self._push(target=self.q_slew, rec=None)
        time.sleep(SEND_DT)

    # ---------- 重复定位测试状态机 ----------
    def _test_step(self):
        r = self._read_and_protect()
        if r is None:
            return
        q, dq, _tau = r
        c = self.cfg
        now = time.time()
        lim = max(0.05, self.pmax - 0.02)

        if self.phase in (PH_TO_A, PH_DWELL_A):
            target = c["pos_a"]
            clamp = 0.0
        else:
            target = c["pos_b"]
            # 合拢端在停留时施加加持力(向合拢方向, 即负力矩)
            clamp = -abs(c["grip_force"]) if self.phase == PH_DWELL_B else 0.0
        target = max(-lim, min(lim, target))

        # 目标限速平滑
        max_slew = max(1e-4, c["speed"]) * SEND_DT
        self.q_slew += max(-max_slew, min(max_slew, target - self.q_slew))
        self.mc.controlMIT(self.motor, c["kp"], c["kd"], self.q_slew, 0.0, clamp)
        self._push(target=target, rec=None)

        settled = (abs(q - target) < c["tol"] and abs(dq) < c["vel_settle"]
                   and abs(self.q_slew - target) < c["tol"])
        timeout = (now - self.phase_start) > c["move_timeout"]

        if self.phase == PH_TO_A:
            if settled or timeout:
                self.phase = PH_DWELL_A
                self.dwell_start = now
        elif self.phase == PH_DWELL_A:
            if now - self.dwell_start >= c["dwell"]:
                self._record("A", q)
                self.phase = PH_TO_B
                self.phase_start = now
        elif self.phase == PH_TO_B:
            if settled or timeout:
                self.phase = PH_DWELL_B
                self.dwell_start = now
        elif self.phase == PH_DWELL_B:
            if now - self.dwell_start >= c["dwell"]:
                self._record("B", q)
                self.cycle += 1
                if self.cycle >= int(c["repeats"]):
                    self.test_running = False
                    self.phase = PH_DONE
                    self.state = "完成"
                    self.reason = f"已完成 {self.cycle} 次往返"
                else:
                    self.phase = PH_TO_A
                    self.phase_start = now
        time.sleep(SEND_DT)

    def _record(self, which, pos):
        with self.lock:
            if which == "A":
                self.samples_a.append((self.cycle, pos))
            else:
                self.samples_b.append((self.cycle, pos))
        # 标记到位时刻 (用于轨迹图打点)
        self._push(target=pos, rec=which)

    def _push(self, target, rec):
        t = time.time() - self.t0
        with self.lock:
            self.buf.append(dict(t=t, q=self._last["q"], target=target, rec=rec))

    def _safe_shutdown(self):
        try:
            self._disable()
        finally:
            if self.serial:
                try:
                    self.serial.close()
                except Exception:
                    pass


def calc_stats(samples):
    """samples: [(cycle, pos)] -> dict(n, mean, sd, rng, rep)。"""
    vals = [p for _c, p in samples]
    n = len(vals)
    if n == 0:
        return dict(n=0, mean=None, sd=None, rng=None, rep=None)
    mean = sum(vals) / n
    sd = statistics.stdev(vals) if n >= 2 else 0.0
    rng = max(vals) - min(vals)
    return dict(n=n, mean=mean, sd=sd, rng=rng, rep=rng / 2.0)


# ==========================================================================
# GUI
# ==========================================================================
class RepeatabilityLab:
    def __init__(self, root):
        self.root = root
        self.worker = RepeatWorker()
        self.worker.start()
        self._alive = True
        self._after_id = None

        root.title("DM4310 夹爪行程重复定位精度测试台")
        try:
            root.state("zoomed")
        except Exception:
            pass
        root.minsize(1120, 700)

        self._build_layout()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh_loop()

    def _build_layout(self):
        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True)

        left_container = ttk.Frame(main, width=372)
        left_container.pack(side="left", fill="y")
        left_container.pack_propagate(False)
        self._lc = tk.Canvas(left_container, highlightthickness=0, width=356)
        vsb = ttk.Scrollbar(left_container, orient="vertical", command=self._lc.yview)
        self._lc.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        self._lc.pack(side="left", fill="both", expand=True)
        self.left = ttk.Frame(self._lc)
        self._lc.create_window((0, 0), window=self.left, anchor="nw")
        self.left.bind("<Configure>",
                       lambda e: self._lc.configure(scrollregion=self._lc.bbox("all")))
        self._lc.bind("<Enter>", lambda e: self._lc.bind_all("<MouseWheel>", self._on_wheel))
        self._lc.bind("<Leave>", lambda e: self._lc.unbind_all("<MouseWheel>"))

        right = ttk.Frame(main)
        right.pack(side="left", fill="both", expand=True)
        self._build_plots(right)
        self._build_controls()

    def _on_wheel(self, e):
        self._lc.yview_scroll(int(-e.delta / 120), "units")

    def _section(self, parent, title):
        f = ttk.LabelFrame(parent, text=title)
        f.pack(fill="x", padx=8, pady=5)
        return f

    def _slider(self, parent, label, lo, hi, init, key, fmt="{:.2f}"):
        row = ttk.Frame(parent)
        row.pack(fill="x", padx=4, pady=2)
        ttk.Label(row, text=label, width=12).pack(side="left")
        val_lbl = ttk.Label(row, text=fmt.format(init), width=7)
        val_lbl.pack(side="right")

        def on_change(v):
            fv = float(v)
            val_lbl.config(text=fmt.format(fv))
            self.worker.set_param(key, fv)
        s = ttk.Scale(row, from_=lo, to=hi, value=init, command=on_change)
        s.pack(side="left", fill="x", expand=True, padx=4)
        self.worker.set_param(key, init)
        return s, val_lbl

    def _build_controls(self):
        # 连接
        conn = self._section(self.left, "连接")
        r1 = ttk.Frame(conn); r1.pack(fill="x", padx=4, pady=3)
        ttk.Label(r1, text="串口").pack(side="left")
        self.e_port = ttk.Entry(r1, width=8); self.e_port.insert(0, DEFAULT_PORT)
        self.e_port.pack(side="left", padx=3)
        ttk.Label(r1, text="波特率").pack(side="left")
        self.e_baud = ttk.Entry(r1, width=8); self.e_baud.insert(0, str(DEFAULT_BAUD))
        self.e_baud.pack(side="left", padx=3)
        r2 = ttk.Frame(conn); r2.pack(fill="x", padx=4, pady=3)
        self.btn_conn = ttk.Button(r2, text="连接", command=self._toggle_conn)
        self.btn_conn.pack(side="left", padx=2)
        ttk.Button(r2, text="使能", command=lambda: self.worker.post("enable")).pack(side="left", padx=2)
        ttk.Button(r2, text="失能", command=lambda: self.worker.post("disable")).pack(side="left", padx=2)

        # 行程端点
        st = self._section(self.left, "行程端点")
        rb = ttk.Frame(st); rb.pack(fill="x", padx=4, pady=2)
        ttk.Button(rb, text="A=当前", command=lambda: self._set_here("pos_a", self.s_a)).pack(side="left", padx=2)
        ttk.Button(rb, text="B=当前", command=lambda: self._set_here("pos_b", self.s_b)).pack(side="left", padx=2)
        self.s_a, _ = self._slider(st, "端点A 张开(rad)", -3.3, 3.3, DEF_POS_A, "pos_a")
        self.s_b, _ = self._slider(st, "端点B 合拢(rad)", -3.3, 3.3, DEF_POS_B, "pos_b")

        # 运动参数
        mv = self._section(self.left, "运动参数")
        self._slider(mv, "速度(rad/s)", 0.2, 8.0, 2.0, "speed", "{:.1f}")
        self._slider(mv, "刚性 Kp", 1.0, 50.0, 12.0, "kp", "{:.1f}")
        self._slider(mv, "阻尼 Kd", 0.0, 5.0, 1.0, "kd", "{:.2f}")
        self._slider(mv, "加持力(Nm)", 0.0, 3.0, 0.0, "grip_force")

        # 测试参数
        tp = self._section(self.left, "测试参数")
        rr = ttk.Frame(tp); rr.pack(fill="x", padx=4, pady=3)
        ttk.Label(rr, text="重复次数").pack(side="left")
        self.sp_rep = ttk.Spinbox(rr, from_=1, to=500, width=6,
                                  command=self._on_repeats)
        self.sp_rep.set(20)
        self.sp_rep.pack(side="left", padx=4)
        self.sp_rep.bind("<KeyRelease>", lambda e: self._on_repeats())
        self._slider(tp, "停留时间(s)", 0.1, 3.0, 0.6, "dwell", "{:.1f}")
        self._slider(tp, "到位容差(rad)", 0.002, 0.05, 0.01, "tol", "{:.3f}")

        # 运行控制
        ctl = self._section(self.left, "运行")
        rc = ttk.Frame(ctl); rc.pack(fill="x", padx=4, pady=4)
        self.btn_start = ttk.Button(rc, text="▶ 开始测试", command=self._start)
        self.btn_start.pack(side="left", padx=2)
        ttk.Button(rc, text="■ 停止", command=lambda: self.worker.post("test_stop")).pack(side="left", padx=2)
        ttk.Button(rc, text="清空数据", command=lambda: self.worker.post("test_reset")).pack(side="left", padx=2)
        rc2 = ttk.Frame(ctl); rc2.pack(fill="x", padx=4, pady=2)
        ttk.Button(rc2, text="导出CSV", command=self._export).pack(side="left", padx=2)
        self.prog_var = tk.StringVar(value="进度: —")
        ttk.Label(ctl, textvariable=self.prog_var, foreground="#0d47a1").pack(anchor="w", padx=6, pady=2)

        # 实时读数
        rt = self._section(self.left, "实时状态")
        self.rt_vars = {k: tk.StringVar(value="—") for k in ("q", "dq", "tau", "tmos", "phase")}
        grid = ttk.Frame(rt); grid.pack(fill="x", padx=4, pady=2)
        for i, (lab, key) in enumerate([("位置 q", "q"), ("速度 dq", "dq"), ("力矩 τ", "tau"),
                                        ("MOS温度", "tmos"), ("相位", "phase")]):
            ttk.Label(grid, text=lab + ":").grid(row=i, column=0, sticky="w", padx=2, pady=1)
            ttk.Label(grid, textvariable=self.rt_vars[key], foreground="#0d47a1",
                      font=("Consolas", 10, "bold")).grid(row=i, column=1, sticky="w", padx=6)

        # 统计结果
        stat = self._section(self.left, "重复定位精度统计")
        self.stat_vars = {}
        head = ttk.Frame(stat); head.pack(fill="x", padx=4)
        for j, h in enumerate(["", "端点A", "端点B"]):
            ttk.Label(head, text=h, width=11, font=("", 9, "bold")).grid(row=0, column=j, padx=2)
        rows = [("n", "样本数"), ("mean", "均值(rad)"), ("sd", "σ(rad)"),
                ("rng", "极差(rad)"), ("rep", "重复精度±")]
        body = ttk.Frame(stat); body.pack(fill="x", padx=4, pady=2)
        for i, (key, lab) in enumerate(rows):
            ttk.Label(body, text=lab, width=11).grid(row=i, column=0, padx=2, sticky="w")
            for j, side in enumerate(("A", "B")):
                v = tk.StringVar(value="—")
                self.stat_vars[f"{key}_{side}"] = v
                ttk.Label(body, textvariable=v, width=11,
                          foreground="#b71c1c").grid(row=i, column=j + 1, padx=2)

        # 保护
        prot = self._section(self.left, "保护")
        self.prot_temp = tk.BooleanVar(value=True)
        ttk.Checkbutton(prot, text="温度保护(超限失能)", variable=self.prot_temp,
                        command=lambda: self.worker.set_param("prot_temp_on", self.prot_temp.get())
                        ).pack(anchor="w", padx=6)
        self._slider(prot, "温度上限(℃)", 50.0, 110.0, 80.0, "temp_limit", "{:.0f}")

        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(self.left, textvariable=self.status_var, foreground="#0d47a1",
                  wraplength=340).pack(fill="x", padx=8, pady=8)

    def _build_plots(self, parent):
        # 顶部图表类型选择
        bar = ttk.Frame(parent)
        bar.pack(fill="x", padx=6, pady=(6, 0))
        ttk.Label(bar, text="下图类型:").pack(side="left")
        self.chart_mode = tk.StringVar(value="正态分布")
        cb = ttk.Combobox(bar, width=22, state="readonly", textvariable=self.chart_mode,
                          values=["正态分布 (直方图+高斯拟合)", "散点趋势 (到位 vs 循环)",
                                  "回归 (漂移检测)"])
        cb.current(0)
        cb.pack(side="left", padx=6)
        cb.bind("<<ComboboxSelected>>", lambda e: setattr(self, "_last_stat_key", None))
        self._last_stat_key = None

        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax_traj = self.fig.add_subplot(211)
        self.ax_stat = self.fig.add_subplot(212)
        self.ax_traj.set_ylabel("位置 (rad)")
        self.ax_traj.set_xlabel("时间 (s)")
        self.ax_traj.grid(True, alpha=0.3)

        self.line_q, = self.ax_traj.plot([], [], "b-", lw=1.0, label="q 实测")
        self.line_tg, = self.ax_traj.plot([], [], "g--", lw=0.9, label="目标")
        self.ax_traj.legend(loc="upper right", fontsize=8)
        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # ---------- 动作 ----------
    def _toggle_conn(self):
        if self.worker.status()["connected"]:
            self.worker.post("disconnect")
            self.btn_conn.config(text="连接")
        else:
            try:
                baud = int(self.e_baud.get().strip())
            except ValueError:
                messagebox.showerror("错误", "波特率无效")
                return
            self.worker.post("connect", self.e_port.get().strip(), baud)
            self.btn_conn.config(text="断开")

    def _set_here(self, key, slider):
        st = self.worker.status()
        if st["connected"]:
            self.worker.set_param(key, st["q"])
            try:
                slider.set(st["q"])
            except Exception:
                pass

    def _on_repeats(self):
        try:
            n = int(float(self.sp_rep.get()))
            self.worker.set_param("repeats", max(1, n))
        except ValueError:
            pass

    def _start(self):
        st = self.worker.status()
        if not st["connected"] or not st["enabled"]:
            messagebox.showinfo("提示", "请先连接并使能电机。")
            return
        self._on_repeats()
        p = self.worker.get_params()
        if abs(p["pos_a"] - p["pos_b"]) < 0.05:
            messagebox.showwarning("行程过小", "端点 A 与 B 太接近，请拉开行程。")
            return
        if not messagebox.askyesno(
                "开始测试",
                f"将在 A={p['pos_a']:.3f} 与 B={p['pos_b']:.3f} 之间往返 {p['repeats']} 次。\n"
                f"速度={p['speed']:.1f} 刚性={p['kp']:.0f} 加持力={p['grip_force']:.2f}Nm\n\n"
                "请确认行程范围安全、不会撞击。开始?"):
            return
        self.worker.post("test_start")

    def _export(self):
        res = self.worker.get_results()
        if not res["samples_a"] and not res["samples_b"]:
            messagebox.showinfo("无数据", "暂无到位样本可导出。")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            filetypes=[("CSV", "*.csv")],
                                            initialfile="repeatability.csv")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as f:
                w = csv.writer(f)
                w.writerow(["cycle", "endpoint", "arrived_pos_rad", "target_rad"])
                for c, pos in res["samples_a"]:
                    w.writerow([c, "A", f"{pos:.6f}", f"{res['pos_a']:.6f}"])
                for c, pos in res["samples_b"]:
                    w.writerow([c, "B", f"{pos:.6f}", f"{res['pos_b']:.6f}"])
            self.status_var.set(f"已导出: {path}")
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("导出失败", str(e))

    # ---------- 刷新 ----------
    def _refresh_loop(self):
        if not self._alive:
            return
        st = self.worker.status()
        data = self.worker.snapshot()
        res = self.worker.get_results()

        if st["connected"]:
            self.rt_vars["q"].set(f"{st['q']:+.4f}")
            self.rt_vars["dq"].set(f"{st['dq']:+.3f}")
            self.rt_vars["tau"].set(f"{st['tau']:+.3f}")
            self.rt_vars["tmos"].set(f"{st['tmos']:.0f}")
            self.rt_vars["phase"].set(st["phase"])
        self.prog_var.set(f"进度: {st['cycle']}/{st['repeats']}  ·  {st['phase']}")

        conn_s = "已连接" if st["connected"] else "未连接"
        en_s = "使能" if st["enabled"] else "失能"
        self.status_var.set(f"{conn_s} · {en_s} · {st['state']}  {st['reason']}")

        self._update_stats(res)
        self._update_plots(data, res)

        if self._alive:
            self._after_id = self.root.after(REFRESH_MS, self._refresh_loop)

    def _update_stats(self, res):
        for side, samples in (("A", res["samples_a"]), ("B", res["samples_b"])):
            s = calc_stats(samples)
            self.stat_vars[f"n_{side}"].set(str(s["n"]))
            if s["n"] == 0:
                for k in ("mean", "sd", "rng", "rep"):
                    self.stat_vars[f"{k}_{side}"].set("—")
                continue
            self.stat_vars[f"mean_{side}"].set(f"{s['mean']:+.4f}")
            self.stat_vars[f"sd_{side}"].set(f"{s['sd']:.5f}")
            self.stat_vars[f"rng_{side}"].set(f"{s['rng']:.5f}")
            self.stat_vars[f"rep_{side}"].set(f"±{s['rep']:.5f}")

    def _update_plots(self, data, res):
        if data:
            ts = [d["t"] for d in data]
            qs = [d["q"] for d in data]
            tg = [d["target"] for d in data]
            self.line_q.set_data(ts, qs)
            self.line_tg.set_data(ts, tg)
            t1 = ts[-1]
            t0 = max(0.0, t1 - WINDOW_S)
            self.ax_traj.set_xlim(t0, max(t1, t0 + 1))
            self.ax_traj.relim(); self.ax_traj.autoscale_view(scalex=False)

        # 仅在样本数或图表类型变化时重绘下图(避免每帧重算直方图/tight_layout)
        mode = self.chart_mode.get()
        key = (mode, len(res["samples_a"]), len(res["samples_b"]))
        if key != self._last_stat_key:
            self._last_stat_key = key
            self.ax_stat.clear()
            self.ax_stat.grid(True, alpha=0.3)
            if mode.startswith("正态分布"):
                self._plot_normal(res)
            elif mode.startswith("回归"):
                self._plot_regression(res)
            else:
                self._plot_scatter(res)
            try:
                self.fig.tight_layout()
            except Exception:
                pass
        try:
            self.canvas.draw()
        except Exception:
            pass

    @staticmethod
    def _gauss(x, mu, sd):
        if sd <= 1e-9:
            return np.zeros_like(x)
        return (1.0 / (sd * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((x - mu) / sd) ** 2)

    def _plot_normal(self, res):
        """到位偏差(pos-均值) 的直方图 + 高斯拟合。两端点居中叠加, 直观比较离散度。"""
        self.ax_stat.set_xlabel("到位偏差 = 到位位置 − 均值 (rad)")
        self.ax_stat.set_ylabel("概率密度")
        plotted = False
        for side, samples, color in (("A", res["samples_a"], "tab:blue"),
                                     ("B", res["samples_b"], "tab:red")):
            vals = np.array([p for _c, p in samples], dtype=float)
            if vals.size < 2:
                continue
            mu = vals.mean()
            sd = vals.std(ddof=1)
            dev = vals - mu
            bins = max(5, min(20, vals.size // 2))
            self.ax_stat.hist(dev, bins=bins, density=True, alpha=0.35, color=color,
                              label=f"{side}: n={vals.size}, σ={sd:.4f}")
            if sd > 1e-9:
                xs = np.linspace(-4 * sd, 4 * sd, 200)
                self.ax_stat.plot(xs, self._gauss(xs, 0.0, sd), color=color, lw=1.8)
                # ±3σ 参考线 (重复精度区间)
                for s in (-3 * sd, 3 * sd):
                    self.ax_stat.axvline(s, color=color, ls=":", lw=0.8)
            plotted = True
        self.ax_stat.set_title("到位偏差正态分布 (虚线=±3σ)", fontsize=9)
        if plotted:
            self.ax_stat.legend(loc="upper right", fontsize=8)
        else:
            self.ax_stat.text(0.5, 0.5, "样本不足 (每端点需≥2次)，开始测试后显示分布",
                              ha="center", va="center", transform=self.ax_stat.transAxes,
                              fontsize=10, color="#888")

    def _plot_scatter(self, res):
        self.ax_stat.set_xlabel("循环序号")
        self.ax_stat.set_ylabel("到位位置 (rad)")
        self.ax_stat.set_title("各次到位位置散点 (虚线=均值)", fontsize=9)
        any_pt = False
        for side, samples, color in (("A", res["samples_a"], "tab:blue"),
                                     ("B", res["samples_b"], "tab:red")):
            if not samples:
                continue
            cs = [c for c, _p in samples]
            ps = [p for _c, p in samples]
            self.ax_stat.scatter(cs, ps, c=color, s=28, label=f"端点{side}到位")
            self.ax_stat.axhline(np.mean(ps), color=color, ls=":", lw=0.8)
            any_pt = True
        if any_pt:
            self.ax_stat.legend(loc="upper right", fontsize=8)

    def _plot_regression(self, res):
        """到位位置 vs 循环序号 + 一次线性回归, 用于检测随循环的系统性漂移。"""
        self.ax_stat.set_xlabel("循环序号")
        self.ax_stat.set_ylabel("到位位置 (rad)")
        self.ax_stat.set_title("到位位置回归 (检测漂移)", fontsize=9)
        any_pt = False
        for side, samples, color in (("A", res["samples_a"], "tab:blue"),
                                     ("B", res["samples_b"], "tab:red")):
            if not samples:
                continue
            cs = np.array([c for c, _p in samples], dtype=float)
            ps = np.array([p for _c, p in samples], dtype=float)
            self.ax_stat.scatter(cs, ps, c=color, s=24)
            label = f"端点{side}"
            if cs.size >= 2 and np.ptp(cs) > 0:
                slope, intercept = np.polyfit(cs, ps, 1)
                xs = np.linspace(cs.min(), cs.max(), 50)
                self.ax_stat.plot(xs, slope * xs + intercept, color=color, lw=1.6)
                label += f": 漂移 {slope * 1000:+.3f} mrad/次"
            self.ax_stat.plot([], [], color=color, label=label)
            any_pt = True
        if any_pt:
            self.ax_stat.legend(loc="upper right", fontsize=8)

    def _on_close(self):
        self._alive = False
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
        self.worker.stop_flag = True
        try:
            self.worker.join(timeout=2.0)
        except Exception:
            pass
        try:
            self.root.quit()
        finally:
            self.root.destroy()


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass
    RepeatabilityLab(root)
    root.mainloop()


if __name__ == "__main__":
    main()
