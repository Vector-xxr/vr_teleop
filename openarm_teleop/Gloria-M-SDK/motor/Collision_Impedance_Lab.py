# -*- coding: utf-8 -*-
"""
Collision / Impedance / Zero Lab —— 达妙 DM4310 碰撞·阻抗·零位 测试台 (GUI)
============================================================================

定位（与其它脚本的区别）
----------------------------------------------------------------------------
- Gripper_Studio.py    : 通用夹爪控制 + 多曲线可视化 + 保护调试工作台。
- Motor_Param_Tool.py  : 电机寄存器参数的离线配置（读/写/保存）。
- 本脚本               : 面向【碰撞等级 / 阻抗 / 零位校准】三类专项测试的
                         “配置 + 控制”一体化测试台，带实时曲线与碰撞标记。

三大测试
----------------------------------------------------------------------------
1) 阻抗测试 (Impedance)
   MIT 模式 controlMIT(kp, kd, q_des, dq_des, tau_ff)：
   - kp = 刚度 (stiffness)，kd = 阻尼 (damping)，q_des = 平衡位置。
   - 手动推动电机，观察偏移与恢复力，调 kp/kd 体会软/硬、欠/过阻尼。

2) 碰撞等级测试 (Collision level)
   软件碰撞检测：按“碰撞等级 1~5”映射到力矩阈值 (Nm)，|实测力矩| 超阈即判定碰撞。
   - 等级越高越灵敏（阈值越小）。阈值可再手动微调。
   - 碰撞响应：报警 / 柔顺让步 / 立即失能 三选一。
   - 统计碰撞次数、记录最近一次碰撞力矩，曲线上以红点标记碰撞时刻。

3) 零位校准 (Zero calibration)
   - 读当前角度；一键“设零位”(set_zero_position) 把当前位置设为 0（带确认）。
   - “回零位”用阻抗控制平滑回到 0。

架构：串口控制环跑在后台 LabWorker 线程（独占串口, 200Hz）；GUI 主线程仅读写
线程安全的共享状态并刷新曲线。所有 Tk 调用只在主线程，串口只在 worker 线程。

运行 (motor/ 目录下)：  python Collision_Impedance_Lab.py
依赖：pyserial, numpy, matplotlib (TkAgg), tkinter
"""
import time
import queue
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
from tkinter import ttk, messagebox

from CAN import (Motor, MotorControl, DM_Motor_Type, DM_variable)
from XuanYa_Comm import MotorComm

# ----------------- 常量 -----------------
DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 921600
SEND_HZ = 200.0
SEND_DT = 1.0 / SEND_HZ
WINDOW_S = 12.0
BUF_LEN = int(SEND_HZ * (WINDOW_S + 3))
REFRESH_MS = 70

# 碰撞等级 -> 力矩阈值 (Nm)。等级越高越灵敏(阈值越小)。DM4310 额定 3Nm/峰值 7Nm。
COLLISION_LEVELS = {1: 3.0, 2: 2.2, 3: 1.5, 4: 1.0, 5: 0.5}

# 碰撞响应
RESP_ALARM = "报警"      # 仅记录/标红, 继续保持
RESP_COMPLY = "柔顺让步"  # 降刚度 + 目标跟随当前位置, 主动让开
RESP_DISABLE = "立即失能"  # 停力矩并失能


class LabWorker(threading.Thread):
    """后台控制线程: 独占串口, 跑 200Hz 阻抗控制环 + 碰撞检测 + 零位校准。"""

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
        self.busy = False  # 校准等占用期, 暂停常规控制环

        # 共享控制参数 (GUI 写, worker 读)
        self.p = dict(
            # 阻抗
            q_des=0.0, kp=8.0, kd=1.0, tau_ff=0.0, speed=2.0,
            # 碰撞
            coll_on=True, coll_level=3, coll_thr=COLLISION_LEVELS[3], coll_resp=RESP_ALARM,
            # 保护
            prot_temp_on=True, temp_limit=80.0,
            tau_ceiling=6.0,   # 硬力矩天花板(前馈钳制), 安全用
        )

        # 内部状态
        self.q_slew = 0.0
        self.pmax = 3.14
        self.state = "NORMAL"   # NORMAL / COLLISION / FAULT
        self.reason = ""
        self.t0 = time.time()
        self.collision_count = 0
        self.last_collision = None   # (t, tau)
        self._in_collision = False
        self._last_q = 0.0
        self._last_dq = 0.0
        self._last_tau = 0.0
        self._last_tmos = 0.0
        self._last_troto = 0.0

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
            return dict(connected=self.connected, enabled=self.enabled, busy=self.busy,
                        state=self.state, reason=self.reason,
                        collision_count=self.collision_count,
                        last_collision=self.last_collision,
                        q=self._last_q, dq=self._last_dq, tau=self._last_tau,
                        tmos=self._last_tmos, troto=self._last_troto,
                        pmax=self.pmax)

    # ---------- 主循环 ----------
    def run(self):
        while not self.stop_flag:
            try:
                self._process_cmds()
                if self.connected and self.enabled and not self.busy:
                    self._control_step()
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
                elif name == "set_zero":
                    self._set_zero()
                elif name == "reset_collision":
                    with self.lock:
                        self.collision_count = 0
                        self.last_collision = None
                    self._in_collision = False
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
        self._disable()
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass
        self.connected = False

    def _enable(self):
        if not self.connected:
            return
        self.mc.disable(self.motor)
        time.sleep(0.3)
        self.mc.enable(self.motor)
        time.sleep(0.1)
        self.mc.refresh_motor_status(self.motor)
        cur = self.comm.getPosRad(self.motor)
        self.q_slew = cur
        with self.lock:
            self.p["q_des"] = cur   # 使能即保持当前位置, 防止跳变
        self.enabled = True
        self.state = "NORMAL"
        self.reason = ""

    def _disable(self):
        if self.connected and self.mc:
            for _ in range(10):
                self.mc.controlMIT(self.motor, 0.0, 0.8, 0.0, 0.0, 0.0)
                time.sleep(SEND_DT)
            self.mc.disable(self.motor)
        self.enabled = False

    def _set_zero(self):
        """把当前位置设为机械零点(set_zero_position)，并把软件目标对齐到 0。"""
        if not self.connected or self.mc is None:
            return
        self.busy = True
        try:
            self.mc.set_zero_position(self.motor)
            time.sleep(0.05)
            self.mc.refresh_motor_status(self.motor)
            self.q_slew = self.comm.getPosRad(self.motor)
            with self.lock:
                self.p["q_des"] = 0.0
            self.reason = "已设零位: 当前位置=0"
        finally:
            self.busy = False

    # ---------- 阻抗控制 + 碰撞检测 ----------
    def _control_step(self):
        p = self.get_params()
        q = self.comm.getPosRad(self.motor)
        dq = self.comm.getVelRad(self.motor)
        tau = self.comm.getTorque(self.motor)
        tmos = float(self.comm.getTMos(self.motor))
        troto = float(self.comm.getTRoto(self.motor))

        # --- 温度保护 (最高优先级) ---
        if p["prot_temp_on"] and (tmos >= p["temp_limit"] or troto >= p["temp_limit"]):
            self.state = "FAULT"
            self.reason = f"过温 Tmos={tmos:.0f} Troto={troto:.0f} >= {p['temp_limit']:.0f}C"
            self.mc.controlMIT(self.motor, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.enabled = False
            self.mc.disable(self.motor)
            self._stash(q, dq, tau, tmos, troto)
            self._push_sample(False)
            return

        # --- 碰撞检测 ---
        thr = p["coll_thr"]
        collided = bool(p["coll_on"]) and abs(tau) > thr
        # 上升沿计数 + 记录
        if collided and not self._in_collision:
            self._in_collision = True
            with self.lock:
                self.collision_count += 1
                self.last_collision = (time.time() - self.t0, tau)
        elif self._in_collision and abs(tau) < thr * 0.7:
            self._in_collision = False

        kp = p["kp"]
        kd = p["kd"]
        q_des = p["q_des"]
        tau_ff = max(-p["tau_ceiling"], min(p["tau_ceiling"], p["tau_ff"]))
        resp = p["coll_resp"]

        # --- 碰撞响应 ---
        if collided and resp == RESP_DISABLE:
            self.state = "FAULT"
            self.reason = f"碰撞触发失能 |tau|={abs(tau):.2f} > {thr:.2f}Nm"
            self.mc.controlMIT(self.motor, 0.0, 0.8, 0.0, 0.0, 0.0)
            self.enabled = False
            self.mc.disable(self.motor)
            self._stash(q, dq, tau, tmos, troto)
            self._push_sample(True)
            return
        if collided and resp == RESP_COMPLY:
            # 柔顺: 目标跟随当前位置, 降刚度, 去前馈 -> 主动让开
            q_des = q
            self.q_slew = q
            kp = max(0.3, kp * 0.2)
            tau_ff = 0.0
            self.state = "COLLISION"
            self.reason = f"柔顺让步 |tau|={abs(tau):.2f} > {thr:.2f}Nm"
        elif collided:  # 报警
            self.state = "COLLISION"
            self.reason = f"碰撞报警 |tau|={abs(tau):.2f} > {thr:.2f}Nm"
        else:
            self.state = "NORMAL"
            self.reason = ""

        # --- 目标限速平滑 + 量程钳制 ---
        lim = max(0.05, self.pmax - 0.02)
        q_des = max(-lim, min(lim, q_des))
        max_slew = max(1e-4, p["speed"]) * SEND_DT
        self.q_slew += max(-max_slew, min(max_slew, q_des - self.q_slew))

        self.mc.controlMIT(self.motor, kp, kd, self.q_slew, 0.0, tau_ff)
        self._stash(q, dq, tau, tmos, troto)
        self._push_sample(collided)
        time.sleep(SEND_DT)

    def _stash(self, q, dq, tau, tmos, troto):
        self._last_q, self._last_dq, self._last_tau = q, dq, tau
        self._last_tmos, self._last_troto = tmos, troto

    def _push_sample(self, collided):
        t = time.time() - self.t0
        with self.lock:
            self.buf.append(dict(t=t, q=self._last_q, dq=self._last_dq, tau=self._last_tau,
                                 q_des=self.q_slew, thr=self.p["coll_thr"],
                                 collided=collided))

    def _safe_shutdown(self):
        try:
            self._disable()
        finally:
            if self.serial:
                try:
                    self.serial.close()
                except Exception:
                    pass


# ==========================================================================
# GUI
# ==========================================================================
class CollisionImpedanceLab:
    def __init__(self, root):
        self.root = root
        self.worker = LabWorker()
        self.worker.start()
        self._alive = True
        self._after_id = None

        root.title("DM4310 碰撞 / 阻抗 / 零位 测试台")
        try:
            root.state("zoomed")
        except Exception:
            pass
        root.minsize(1100, 680)

        self._build_layout()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh_loop()

    # ---------- 布局 ----------
    def _build_layout(self):
        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True)

        # 左侧控制栏(可滚动)
        left_container = ttk.Frame(main, width=360)
        left_container.pack(side="left", fill="y")
        left_container.pack_propagate(False)
        self._lc = tk.Canvas(left_container, highlightthickness=0, width=344)
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

        # 右侧曲线
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
        ttk.Label(row, text=label, width=11).pack(side="left")
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
        self.btn_en = ttk.Button(r2, text="使能", command=lambda: self.worker.post("enable"))
        self.btn_en.pack(side="left", padx=2)
        ttk.Button(r2, text="失能", command=lambda: self.worker.post("disable")).pack(side="left", padx=2)

        # 实时读数
        rt = self._section(self.left, "实时状态 (只读)")
        self.rt_vars = {k: tk.StringVar(value="—")
                        for k in ("q", "dq", "tau", "tmos", "troto", "state", "coll")}
        grid = ttk.Frame(rt); grid.pack(fill="x", padx=4, pady=2)
        items = [("位置 q (rad)", "q"), ("速度 dq", "dq"), ("力矩 τ (Nm)", "tau"),
                 ("MOS温度", "tmos"), ("线圈温度", "troto"),
                 ("状态", "state"), ("碰撞次数", "coll")]
        for i, (lab, key) in enumerate(items):
            ttk.Label(grid, text=lab + ":").grid(row=i, column=0, sticky="w", padx=2, pady=1)
            ttk.Label(grid, textvariable=self.rt_vars[key], foreground="#0d47a1",
                      font=("Consolas", 10, "bold")).grid(row=i, column=1, sticky="w", padx=6)

        # 阻抗测试
        imp = self._section(self.left, "① 阻抗测试 (MIT)")
        rb = ttk.Frame(imp); rb.pack(fill="x", padx=4, pady=2)
        ttk.Button(rb, text="保持当前位置", command=self._hold_here).pack(side="left", padx=2)
        ttk.Button(rb, text="回零位", command=lambda: self.worker.set_param("q_des", 0.0)).pack(side="left", padx=2)
        self.s_qdes, _ = self._slider(imp, "平衡位置(rad)", -3.14, 3.14, 0.0, "q_des")
        self._slider(imp, "刚度 Kp", 0.0, 50.0, 8.0, "kp", "{:.1f}")
        self._slider(imp, "阻尼 Kd", 0.0, 5.0, 1.0, "kd", "{:.2f}")
        self._slider(imp, "前馈力矩", -3.0, 3.0, 0.0, "tau_ff")
        self._slider(imp, "速度(rad/s)", 0.2, 8.0, 2.0, "speed", "{:.1f}")

        # 碰撞等级测试
        col = self._section(self.left, "② 碰撞等级测试")
        rc = ttk.Frame(col); rc.pack(fill="x", padx=4, pady=2)
        self.coll_on = tk.BooleanVar(value=True)
        ttk.Checkbutton(rc, text="启用碰撞检测", variable=self.coll_on,
                        command=lambda: self.worker.set_param("coll_on", self.coll_on.get())
                        ).pack(side="left")
        rl = ttk.Frame(col); rl.pack(fill="x", padx=4, pady=2)
        ttk.Label(rl, text="碰撞等级(1迟钝~5灵敏)").pack(side="left")
        self.coll_level = tk.IntVar(value=3)
        self.cb_level = ttk.Combobox(rl, width=5, state="readonly",
                                     values=list(COLLISION_LEVELS.keys()),
                                     textvariable=self.coll_level)
        self.cb_level.pack(side="left", padx=4)
        self.cb_level.bind("<<ComboboxSelected>>", self._on_level)
        self.lbl_thr = ttk.Label(col, text=f"映射阈值: {COLLISION_LEVELS[3]:.2f} Nm",
                                 foreground="#b71c1c")
        self.lbl_thr.pack(anchor="w", padx=6)
        self.s_thr, self.s_thr_lbl = self._slider(col, "阈值微调(Nm)", 0.1, 7.0,
                                                  COLLISION_LEVELS[3], "coll_thr")
        rr = ttk.Frame(col); rr.pack(fill="x", padx=4, pady=2)
        ttk.Label(rr, text="碰撞响应").pack(side="left")
        self.coll_resp = tk.StringVar(value=RESP_ALARM)
        for txt in (RESP_ALARM, RESP_COMPLY, RESP_DISABLE):
            ttk.Radiobutton(rr, text=txt, value=txt, variable=self.coll_resp,
                            command=lambda: self.worker.set_param("coll_resp", self.coll_resp.get())
                            ).pack(side="left")
        rk = ttk.Frame(col); rk.pack(fill="x", padx=4, pady=2)
        self.lbl_lastcoll = ttk.Label(rk, text="最近碰撞: —")
        self.lbl_lastcoll.pack(side="left")
        ttk.Button(rk, text="清零计数", command=lambda: self.worker.post("reset_collision")
                   ).pack(side="right")

        # 零位校准
        zero = self._section(self.left, "③ 零位校准")
        ttk.Label(zero, text="把当前位置设为机械零点。注意会平移坐标系，",
                  wraplength=320, foreground="#555").pack(anchor="w", padx=6)
        ttk.Label(zero, text="设零后阻抗目标自动对齐到 0。",
                  wraplength=320, foreground="#555").pack(anchor="w", padx=6)
        rz = ttk.Frame(zero); rz.pack(fill="x", padx=4, pady=4)
        ttk.Button(rz, text="● 设零位 (set_zero)", command=self._do_zero).pack(side="left", padx=2)
        ttk.Button(rz, text="回零位", command=lambda: self.worker.set_param("q_des", 0.0)
                   ).pack(side="left", padx=2)

        # 保护
        prot = self._section(self.left, "保护")
        self.prot_temp = tk.BooleanVar(value=True)
        ttk.Checkbutton(prot, text="温度保护(超限失能)", variable=self.prot_temp,
                        command=lambda: self.worker.set_param("prot_temp_on", self.prot_temp.get())
                        ).pack(anchor="w", padx=6)
        self._slider(prot, "温度上限(℃)", 50.0, 110.0, 80.0, "temp_limit", "{:.0f}")
        self._slider(prot, "力矩天花板(Nm)", 0.5, 7.0, 6.0, "tau_ceiling", "{:.1f}")

        # 状态栏
        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(self.left, textvariable=self.status_var, foreground="#0d47a1",
                  wraplength=330).pack(fill="x", padx=8, pady=8)

    def _build_plots(self, parent):
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax_q = self.fig.add_subplot(211)
        self.ax_tau = self.fig.add_subplot(212, sharex=self.ax_q)
        self.ax_q.set_ylabel("位置 (rad)")
        self.ax_q.grid(True, alpha=0.3)
        self.ax_tau.set_ylabel("力矩 (Nm)")
        self.ax_tau.set_xlabel("时间 (s)")
        self.ax_tau.grid(True, alpha=0.3)

        self.line_q, = self.ax_q.plot([], [], "b-", lw=1.2, label="q 实测")
        self.line_qd, = self.ax_q.plot([], [], "g--", lw=1.0, label="q 目标")
        self.line_tau, = self.ax_tau.plot([], [], "m-", lw=1.2, label="τ 实测")
        self.thr_hi = self.ax_tau.axhline(0, color="r", ls=":", lw=1.0, label="碰撞阈值")
        self.thr_lo = self.ax_tau.axhline(0, color="r", ls=":", lw=1.0)
        self.coll_scatter = self.ax_tau.scatter([], [], c="red", s=24, zorder=5, label="碰撞")
        self.ax_q.legend(loc="upper right", fontsize=8)
        self.ax_tau.legend(loc="upper right", fontsize=8)
        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # ---------- 动作 ----------
    def _toggle_conn(self):
        st = self.worker.status()
        if st["connected"]:
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

    def _hold_here(self):
        st = self.worker.status()
        if st["connected"]:
            self.worker.set_param("q_des", st["q"])
            try:
                self.s_qdes.set(st["q"])
            except Exception:
                pass

    def _on_level(self, _evt=None):
        lvl = int(self.coll_level.get())
        thr = COLLISION_LEVELS[lvl]
        self.worker.set_param("coll_level", lvl)
        self.worker.set_param("coll_thr", thr)
        self.lbl_thr.config(text=f"映射阈值: {thr:.2f} Nm")
        try:
            self.s_thr.set(thr)
            self.s_thr_lbl.config(text=f"{thr:.2f}")
        except Exception:
            pass

    def _do_zero(self):
        if not self.worker.status()["connected"]:
            messagebox.showinfo("未连接", "请先连接电机。")
            return
        if messagebox.askyesno("设零位确认",
                               "将把当前位置设为机械零点(0)。\n这会平移坐标系，"
                               "之前记录的位置含义随之改变。\n\n确认设零位?"):
            self.worker.post("set_zero")

    # ---------- 刷新 ----------
    def _refresh_loop(self):
        if not self._alive:
            return
        st = self.worker.status()
        data = self.worker.snapshot()

        # 实时读数
        if st["connected"]:
            self.rt_vars["q"].set(f"{st['q']:+.4f}")
            self.rt_vars["dq"].set(f"{st['dq']:+.3f}")
            self.rt_vars["tau"].set(f"{st['tau']:+.3f}")
            self.rt_vars["tmos"].set(f"{st['tmos']:.0f}")
            self.rt_vars["troto"].set(f"{st['troto']:.0f}")
            self.rt_vars["state"].set(st["state"])
            self.rt_vars["coll"].set(str(st["collision_count"]))
        lc = st["last_collision"]
        self.lbl_lastcoll.config(
            text=f"最近碰撞: t={lc[0]:.1f}s τ={lc[1]:+.2f}Nm" if lc else "最近碰撞: —")

        conn_s = "已连接" if st["connected"] else "未连接"
        en_s = "使能" if st["enabled"] else "失能"
        self.status_var.set(f"{conn_s} · {en_s} · {st['state']}  {st['reason']}")

        self._update_plots(data, st)

        # 同步绘制完成后再排程下一次, 防止事件循环饱和
        if self._alive:
            self._after_id = self.root.after(REFRESH_MS, self._refresh_loop)

    def _update_plots(self, data, st):
        if data:
            ts = [d["t"] for d in data]
            qs = [d["q"] for d in data]
            qds = [d["q_des"] for d in data]
            taus = [d["tau"] for d in data]
            self.line_q.set_data(ts, qs)
            self.line_qd.set_data(ts, qds)
            self.line_tau.set_data(ts, taus)
            t1 = ts[-1]
            t0 = max(0.0, t1 - WINDOW_S)
            self.ax_q.set_xlim(t0, max(t1, t0 + 1))
            self.ax_q.relim(); self.ax_q.autoscale_view(scalex=False)
            self.ax_tau.relim(); self.ax_tau.autoscale_view(scalex=False)
            # 碰撞标记
            cx = [d["t"] for d in data if d["collided"]]
            cy = [d["tau"] for d in data if d["collided"]]
            if cx:
                self.coll_scatter.set_offsets(np.column_stack([cx, cy]))
            else:
                self.coll_scatter.set_offsets(np.empty((0, 2)))

        thr = self.worker.get_params()["coll_thr"]
        self.thr_hi.set_ydata([thr, thr])
        self.thr_lo.set_ydata([-thr, -thr])
        try:
            self.canvas.draw()
        except Exception:
            pass

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
    CollisionImpedanceLab(root)
    root.mainloop()


if __name__ == "__main__":
    main()
