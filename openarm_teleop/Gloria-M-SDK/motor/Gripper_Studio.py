# -*- coding: utf-8 -*-
"""
Gripper Studio - 达妙 DM4310 夹爪控制与可视化调试工作站
=======================================================
专业的单文件调试工具, 集成:
  - 实时曲线: 位置(开度%)、速度、力矩/电流、温度 (matplotlib 嵌入 Tkinter)
  - 控制面板: 位置(行程)模式 / 夹持力模式 / MIT 原始模式; 一键开/合/中
  - 保护调试: 速度上限 / 力矩上限 / 温度上限 (可开关), 触发即保护并在曲线上标红
  - 行程标定 / 设零位 / 使能-失能 / CSV 数据记录
架构: 串口控制环跑在后台线程 (独占串口), GUI 只读写线程安全的共享状态。

运行 (motor/ 目录下):
    python Gripper_Studio.py
依赖: pyserial, numpy, matplotlib (TkAgg), tkinter
"""
import time
import csv
import queue
import threading
import collections

import serial
import matplotlib
matplotlib.use("TkAgg")
# 配置中文字体, 否则曲线坐标轴中文会显示为方框
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

import tkinter as tk
from tkinter import ttk, messagebox

from CAN import *
from XuanYa_Comm import MotorComm, gloria_max

# ----------------- 常量 (来自实测标定) -----------------
DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 921600
OPEN_END = -0.35          # 全开端 (留余量)
CLOSE_END = -2.70         # 全合端 (留余量)
MID = (OPEN_END + CLOSE_END) / 2.0

SEND_HZ = 200.0
SEND_DT = 1.0 / SEND_HZ
WINDOW_S = 12.0           # 曲线时间窗 (s)
BUF_LEN = int(SEND_HZ * (WINDOW_S + 3))
REFRESH_MS = 70           # 两次刷新之间的固定空闲间隔 (ms); 因采用同步绘制, 这是绘制完成后的等待

# 模式
MODE_POSITION = "POSITION"
MODE_FORCE = "FORCE"
MODE_MIT = "MIT_RAW"


class MotorWorker(threading.Thread):
    """后台控制线程: 独占串口, 跑 200Hz 控制环 + 保护, 输出遥测。"""

    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.cmd_q = queue.Queue()
        self.buf = collections.deque(maxlen=BUF_LEN)
        self.stop_flag = False

        # 设备
        self.serial = None
        self.mc = None
        self.motor = None
        self.comm = MotorComm()
        self.connected = False
        self.enabled = False
        self.calibrating = False

        # 共享控制参数 (GUI 写, worker 读)
        self.p = dict(
            mode=MODE_POSITION,
            # position (speed=位置模式行程速度 rad/s, 通过限速平滑限制目标移动速率)
            q_goal=MID, kp_pos=12.0, kd_pos=1.0, speed=2.4,
            # force
            grip_force=0.3, gripping=False, kd_force=0.8,
            # mit raw
            mit_kp=0.0, mit_kd=0.5, mit_q=MID, mit_dq=0.0, mit_tau=0.0,
            # protection
            prot_vel_on=True, vel_limit=8.0,
            prot_tau_on=True, tau_limit=2.0,
            prot_temp_on=True, temp_limit=80.0,
            # 电气: KT 力矩常数 (Nm/A), 用于由力矩估算相电流 I=tau/KT
            # DM4310 额定 3Nm/2.5A -> ~1.2 Nm/A (KT_Value 未标定时的缺省估计)
            kt=1.2,
        )
        # 内部状态
        self.q_slew = MID
        self.prot_state = "NORMAL"   # NORMAL / PROTECT / FAULT
        self.prot_reason = ""
        self.t0 = time.time()
        # 电气参数 (静态, 读自电机): UV欠压 OV过压 OC过流 KT
        self.elec = dict(UV=None, OV=None, OC=None, KT=None)
        # 行程限位版本号: 每次重标定 +1, 供 GUI 同步滑条范围
        self.limits_version = 0

        # CSV
        self.csv_writer = None
        self.csv_file = None
        self.csv_lock = threading.Lock()

    # ---------- GUI 调用的接口 ----------
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
                        calibrating=self.calibrating,
                        prot_state=self.prot_state, prot_reason=self.prot_reason,
                        limits_version=self.limits_version)

    def get_elec(self):
        with self.lock:
            return dict(self.elec)

    # ---------- 线程主循环 ----------
    def run(self):
        while not self.stop_flag:
            try:
                self._process_cmds()
                if self.connected and self.enabled and not self.calibrating:
                    self._control_step()
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.prot_reason = f"loop err: {e}"
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
                elif name == "zero":
                    if self.mc:
                        self.mc.set_zero_position(self.motor)
                elif name == "calibrate":
                    self._calibrate()
                elif name == "selfcheck":
                    self._selfcheck()
                elif name == "csv_start":
                    self._csv_start(cmd[1])
                elif name == "csv_stop":
                    self._csv_stop()
                elif name == "read_elec":
                    self._read_elec()
            except Exception as e:
                self.prot_reason = f"cmd {name} err: {e}"

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
        self._read_elec()

    def _sync_limits(self):
        """读取电机实际 PMAX/VMAX/TMAX 并同步到 SDK Limit_Param, 修正 VMAX 量纲。
        返回 (pmax, vmax, tmax)。"""
        pmax = self.mc.read_motor_param(self.motor, DM_variable.PMAX)
        vmax = self.mc.read_motor_param(self.motor, DM_variable.VMAX)
        tmax = self.mc.read_motor_param(self.motor, DM_variable.TMAX)
        if pmax and vmax and tmax:
            self.mc.change_limit_param(DM_Motor_Type.DM4310, pmax, vmax, tmax)
        return pmax, vmax, tmax

    def _read_elec(self):
        """读取电机静态电气参数 (欠压/过压/过流阈值, KT)。注意: 这些是配置阈值,
        DM4310 CAN 反馈帧不含实时母线电压/相电流, 实时电流只能由力矩估算。"""
        if not self.connected or self.mc is None:
            return
        uv = self.mc.read_motor_param(self.motor, DM_variable.UV_Value)
        ov = self.mc.read_motor_param(self.motor, DM_variable.OV_Value)
        oc = self.mc.read_motor_param(self.motor, DM_variable.OC_Value)
        kt = self.mc.read_motor_param(self.motor, DM_variable.KT_Value)
        with self.lock:
            self.elec = dict(UV=uv, OV=ov, OC=oc, KT=kt)
            # 若电机已标定 KT(>0) 则采用之, 否则保留缺省估计
            if kt and kt > 1e-6:
                self.p["kt"] = float(kt)

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
        self.q_slew = self.comm.getPosRad(self.motor)
        with self.lock:
            self.p["q_goal"] = max(CLOSE_END, min(OPEN_END, self.q_slew))
        self.enabled = True
        self.prot_state = "NORMAL"
        self.prot_reason = ""

    def _disable(self):
        if self.connected and self.mc:
            for _ in range(10):
                self.mc.controlMIT(self.motor, 0.0, 0.8, 0.0, 0.0, 0.0)
                time.sleep(SEND_DT)
            self.mc.disable(self.motor)
        self.enabled = False

    def _calibrate(self):
        """小力矩试探两端机械限位, 重新锚定软件行程限位到物理限位。"""
        self.calibrating = True
        try:
            if not self.enabled:
                self._enable()
            global OPEN_END, CLOSE_END, MID
            q_neg = self._probe(-1.0)
            time.sleep(0.2)
            q_pos = self._probe(+1.0)
            lo, hi = min(q_neg, q_pos), max(q_neg, q_pos)
            CLOSE_END = lo + 0.03
            OPEN_END = hi - 0.03
            MID = (OPEN_END + CLOSE_END) / 2.0
            # 重锚定后, 把软件状态对齐到当前实际位置并复位保护, 避免残留旧目标
            cur = self.comm.getPosRad(self.motor)
            self.q_slew = cur
            with self.lock:
                self.p["q_goal"] = max(CLOSE_END, min(OPEN_END, cur))
                self.p["mode"] = MODE_POSITION
                self.p["gripping"] = False
                self.limits_version += 1
            self.prot_state = "NORMAL"
            self.prot_reason = f"标定: OPEN={OPEN_END:.3f} CLOSE={CLOSE_END:.3f}"
        finally:
            self.calibrating = False

    def _selfcheck(self):
        """初始化自检: 恢复电机与软件一致性。
        步骤: 通讯/量程同步 -> 读电气参数 -> 使能 -> 重新标定行程(重锚定限位)
        -> 复位模式/目标/保护 -> 回到中点。用于误标零或状态错乱后的恢复。"""
        if not self.connected or self.mc is None:
            self.prot_reason = "自检失败: 未连接"
            return
        self.calibrating = True
        try:
            self._sync_limits()
            self._read_elec()
            if not self.enabled:
                self._enable()
        finally:
            self.calibrating = False
        # 重新标定 (内部自带 calibrating 标志与限位重锚定)
        self._calibrate()
        # 回到中点, 进入位置模式, 保护复位
        with self.lock:
            self.p["mode"] = MODE_POSITION
            self.p["q_goal"] = MID
            self.p["gripping"] = False
        self.prot_state = "NORMAL"
        self.prot_reason = f"自检完成: 行程重锚定 OPEN={OPEN_END:.3f} CLOSE={CLOSE_END:.3f}, 已回中点"

    def _probe(self, sign, probe_tau=0.5, kd=0.9, vel_stall=0.05,
               stall_hold=0.5, min_drive=1.2, timeout=12.0):
        t0 = time.time()
        stall_since = None
        q = self.comm.getPosRad(self.motor)
        while time.time() - t0 < timeout:
            now = time.time()
            self.mc.controlMIT(self.motor, 0.0, kd, 0.0, 0.0, sign * probe_tau)
            dq = self.comm.getVelRad(self.motor)
            q = self.comm.getPosRad(self.motor)
            self._push_sample(sign * probe_tau)
            if abs(dq) > 10.0:
                break
            if abs(dq) < vel_stall:
                if stall_since is None:
                    stall_since = now
                elif (now - stall_since) >= stall_hold and (now - t0) >= min_drive:
                    break
            else:
                stall_since = None
            time.sleep(SEND_DT)
        return q

    # ---------- 控制 + 保护 ----------
    def _control_step(self):
        p = self.get_params()
        q = self.comm.getPosRad(self.motor)
        dq = self.comm.getVelRad(self.motor)
        tau = self.comm.getTorque(self.motor)
        tmos = float(self.comm.getTMos(self.motor))
        troto = float(self.comm.getTRoto(self.motor))

        prot = "NORMAL"
        reason = ""

        # --- 温度保护 (最高优先级, 直接失能) ---
        if p["prot_temp_on"] and (tmos >= p["temp_limit"] or troto >= p["temp_limit"]):
            prot = "FAULT"
            reason = f"过温 Tmos={tmos:.0f} Troto={troto:.0f} >= {p['temp_limit']:.0f}C"
            self.prot_state, self.prot_reason = prot, reason
            self.mc.controlMIT(self.motor, 0.0, 0.0, 0.0, 0.0, 0.0)
            self._push_sample(0.0)
            self.enabled = False
            self.mc.disable(self.motor)
            return

        # --- 速度保护: 超速则切到位置保持 ---
        if p["prot_vel_on"] and abs(dq) > p["vel_limit"]:
            prot = "PROTECT"
            reason = f"超速 |v|={abs(dq):.2f} > {p['vel_limit']:.1f}"
            with self.lock:
                self.p["mode"] = MODE_POSITION
                self.p["q_goal"] = q
                self.p["gripping"] = False
            self.q_slew = q
            p = self.get_params()

        mode = p["mode"]
        tau_cmd_log = 0.0

        if mode == MODE_POSITION:
            goal = max(CLOSE_END, min(OPEN_END, p["q_goal"]))
            # 力矩保护: 过载方向冻结目标
            if p["prot_tau_on"] and abs(tau) > p["tau_limit"]:
                prot = "PROTECT" if prot == "NORMAL" else prot
                reason = reason or f"力矩超限 |tau|={abs(tau):.2f} > {p['tau_limit']:.1f}"
                # tau>0 时阻止继续向正(张开), tau<0 阻止继续向负(闭合)
                if tau > 0 and goal > q:
                    goal = q
                elif tau < 0 and goal < q:
                    goal = q
            max_slew = max(1e-4, p["speed"]) * SEND_DT   # 由速度(rad/s)换算每周期最大步长
            step = max(-max_slew, min(max_slew, goal - self.q_slew))
            self.q_slew += step
            self.mc.controlMIT(self.motor, p["kp_pos"], p["kd_pos"], self.q_slew, 0.0, 0.0)

        elif mode == MODE_FORCE:
            grip = p["grip_force"]
            if p["prot_tau_on"] and grip > p["tau_limit"]:
                grip = p["tau_limit"]
                prot = "PROTECT" if prot == "NORMAL" else prot
                reason = reason or f"夹持力受限于 tau_limit={p['tau_limit']:.1f}"
            tau_cmd_log = -grip if p["gripping"] else 0.0
            self.mc.controlMIT(self.motor, 0.0, p["kd_force"], 0.0, 0.0, tau_cmd_log)

        else:  # MIT_RAW
            tau_raw = p["mit_tau"]
            if p["prot_tau_on"]:
                tau_raw = max(-p["tau_limit"], min(p["tau_limit"], tau_raw))
            tau_cmd_log = tau_raw
            self.mc.controlMIT(self.motor, p["mit_kp"], p["mit_kd"], p["mit_q"], p["mit_dq"], tau_raw)

        self.prot_state, self.prot_reason = prot, reason
        self._push_sample(tau_cmd_log)
        time.sleep(SEND_DT)

    def _push_sample(self, tau_cmd):
        t = time.time() - self.t0
        q = self.comm.getPosRad(self.motor)
        pct = self.comm.getAngle100(self.motor)
        dq = self.comm.getVelRad(self.motor)
        tau = self.comm.getTorque(self.motor)
        tmos = float(self.comm.getTMos(self.motor))
        troto = float(self.comm.getTRoto(self.motor))
        prot = 1.0 if self.prot_state != "NORMAL" else 0.0
        sample = (t, pct, q, dq, tau, tau_cmd, tmos, troto, prot)
        with self.lock:
            self.buf.append(sample)
        with self.csv_lock:
            if self.csv_writer:
                self.csv_writer.writerow([f"{v:.4f}" if isinstance(v, float) else v for v in sample])

    def _csv_start(self, path):
        with self.csv_lock:
            self._csv_stop_locked()
            self.csv_file = open(path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["t", "pct", "q_rad", "dq", "tau", "tau_cmd", "Tmos", "Troto", "prot"])

    def _csv_stop(self):
        with self.csv_lock:
            self._csv_stop_locked()

    def _csv_stop_locked(self):
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass
        self.csv_file = None
        self.csv_writer = None

    def _safe_shutdown(self):
        try:
            self._disable()
        except Exception:
            pass
        self._csv_stop()
        if self.serial:
            try:
                self.serial.close()
            except Exception:
                pass


class GripperStudio:
    def __init__(self, root):
        self.root = root
        self.root.title("Gripper Studio - DM4310 夹爪控制与可视化调试")
        self.root.geometry("1360x820")
        self.root.minsize(1000, 600)
        try:
            self.root.state("zoomed")   # Windows 启动即最大化
        except Exception:
            pass
        self.worker = MotorWorker()
        self.worker.start()

        self._alive = True
        self._after_id = None
        self._last_limits_version = -1
        self._build_layout()
        self._build_controls()
        self._build_plots()
        self._refresh_loop()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- 布局 ----------
    def _build_layout(self):
        # 左侧: 固定宽度容器 + 可垂直滚动的画布, 内容超出窗口高度时可滚动
        LEFT_W = 320
        left_container = ttk.Frame(self.root, width=LEFT_W)
        left_container.pack(side=tk.LEFT, fill=tk.Y)
        left_container.pack_propagate(False)

        self._left_canvas = tk.Canvas(left_container, highlightthickness=0, borderwidth=0)
        vsb = ttk.Scrollbar(left_container, orient="vertical", command=self._left_canvas.yview)
        self._left_canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        self._left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 真正放控件的内部 Frame
        self.left = ttk.Frame(self._left_canvas, padding=8)
        self._left_window = self._left_canvas.create_window((0, 0), window=self.left, anchor="nw")
        self.left.bind(
            "<Configure>",
            lambda e: self._left_canvas.configure(scrollregion=self._left_canvas.bbox("all")))
        self._left_canvas.bind(
            "<Configure>",
            lambda e: self._left_canvas.itemconfig(self._left_window, width=e.width))
        # 鼠标滚轮 (仅当指针位于左栏时滚动, 避免抢占曲线区)
        self.root.bind_all("<MouseWheel>", self._on_mousewheel)

        self.right = ttk.Frame(self.root, padding=4)
        self.right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    def _on_mousewheel(self, event):
        w = self.root.winfo_containing(event.x_root, event.y_root)
        while w is not None:
            if w is self._left_canvas or w is self.left:
                self._left_canvas.yview_scroll(int(-event.delta / 120), "units")
                return
            w = getattr(w, "master", None)

    def _section(self, parent, title):
        lf = ttk.LabelFrame(parent, text=title, padding=6)
        lf.pack(fill=tk.X, pady=4)
        return lf

    def _slider(self, parent, label, frm, to, init, key, fmt="{:.2f}", res=0.01):
        row = ttk.Frame(parent); row.pack(fill=tk.X, pady=1)
        ttk.Label(row, text=label, width=10).pack(side=tk.LEFT)
        val = tk.DoubleVar(value=init)
        lbl = ttk.Label(row, width=7, text=fmt.format(init))
        lbl.pack(side=tk.RIGHT)

        def on_move(v):
            fv = float(v)
            lbl.config(text=fmt.format(fv))
            self.worker.set_param(key, fv)
        s = ttk.Scale(row, from_=frm, to=to, value=init, command=on_move)
        s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        return s, val

    def _build_controls(self):
        # 连接
        conn = self._section(self.left, "连接 / 使能")
        r = ttk.Frame(conn); r.pack(fill=tk.X)
        ttk.Label(r, text="端口").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        ttk.Entry(r, textvariable=self.port_var, width=8).pack(side=tk.LEFT, padx=3)
        self.btn_conn = ttk.Button(r, text="连接", command=self._toggle_conn)
        self.btn_conn.pack(side=tk.LEFT, padx=3)
        r2 = ttk.Frame(conn); r2.pack(fill=tk.X, pady=3)
        self.btn_en = ttk.Button(r2, text="使能", command=self._toggle_enable, state=tk.DISABLED)
        self.btn_en.pack(side=tk.LEFT, padx=3)
        ttk.Button(r2, text="设零位", command=self._do_zero).pack(side=tk.LEFT, padx=3)
        ttk.Button(r2, text="标定行程", command=lambda: self.worker.post("calibrate")).pack(side=tk.LEFT, padx=3)
        # 初始化自检: 恢复电机与软件一致性 (误标零/状态错乱后使用)
        self.btn_selfcheck = ttk.Button(conn, text="● 初始化自检 (恢复一致性)", command=self._do_selfcheck)
        self.btn_selfcheck.pack(fill=tk.X, pady=(3, 0))

        # 模式
        modef = self._section(self.left, "控制模式")
        self.mode_var = tk.StringVar(value=MODE_POSITION)
        for txt, val in [("位置/行程", MODE_POSITION), ("夹持力", MODE_FORCE), ("MIT 原始", MODE_MIT)]:
            ttk.Radiobutton(modef, text=txt, variable=self.mode_var, value=val,
                            command=self._on_mode).pack(anchor=tk.W)

        # 位置控制
        posf = self._section(self.left, "位置 / 行程")
        rr = ttk.Frame(posf); rr.pack(fill=tk.X)
        ttk.Button(rr, text="全开", width=6, command=lambda: self._set_goal(OPEN_END)).pack(side=tk.LEFT, padx=2)
        ttk.Button(rr, text="中点", width=6, command=lambda: self._set_goal(MID)).pack(side=tk.LEFT, padx=2)
        ttk.Button(rr, text="全合", width=6, command=lambda: self._set_goal(CLOSE_END)).pack(side=tk.LEFT, padx=2)
        self.s_goal, _ = self._slider(posf, "目标(rad)", CLOSE_END, OPEN_END, MID, "q_goal", "{:.2f}")
        self._slider(posf, "速度(rad/s)", 0.2, 8.0, 2.4, "speed", "{:.2f}")
        self._slider(posf, "Kp", 0, 50, 12.0, "kp_pos", "{:.1f}")
        self._slider(posf, "Kd", 0, 5, 1.0, "kd_pos", "{:.2f}")

        # 夹持力
        forcef = self._section(self.left, "夹持力")
        self._slider(forcef, "目标力(Nm)", 0, 3.0, 0.3, "grip_force", "{:.2f}")
        self._slider(forcef, "阻尼Kd", 0, 3, 0.8, "kd_force", "{:.2f}")
        rg = ttk.Frame(forcef); rg.pack(fill=tk.X, pady=2)
        ttk.Button(rg, text="夹持", command=lambda: self.worker.set_param("gripping", True)).pack(side=tk.LEFT, padx=3)
        ttk.Button(rg, text="松开", command=lambda: self.worker.set_param("gripping", False)).pack(side=tk.LEFT, padx=3)

        # MIT 原始
        mitf = self._section(self.left, "MIT 原始")
        self._slider(mitf, "Kp", 0, 50, 0.0, "mit_kp", "{:.1f}")
        self._slider(mitf, "Kd", 0, 5, 0.5, "mit_kd", "{:.2f}")
        self.s_mit_q, _ = self._slider(mitf, "q(rad)", CLOSE_END, OPEN_END, MID, "mit_q", "{:.2f}")
        self._slider(mitf, "tau(Nm)", -3.0, 3.0, 0.0, "mit_tau", "{:.2f}")

        # 保护
        protf = self._section(self.left, "保护设置")
        self.prot_vel = tk.BooleanVar(value=True)
        self.prot_tau = tk.BooleanVar(value=True)
        self.prot_temp = tk.BooleanVar(value=True)
        ttk.Checkbutton(protf, text="速度保护", variable=self.prot_vel,
                        command=lambda: self.worker.set_param("prot_vel_on", self.prot_vel.get())).pack(anchor=tk.W)
        self._slider(protf, "vel上限", 1, 20, 8.0, "vel_limit", "{:.1f}")
        ttk.Checkbutton(protf, text="力矩保护", variable=self.prot_tau,
                        command=lambda: self.worker.set_param("prot_tau_on", self.prot_tau.get())).pack(anchor=tk.W)
        self._slider(protf, "tau上限", 0.2, 7, 2.0, "tau_limit", "{:.2f}")
        ttk.Checkbutton(protf, text="温度保护", variable=self.prot_temp,
                        command=lambda: self.worker.set_param("prot_temp_on", self.prot_temp.get())).pack(anchor=tk.W)
        self._slider(protf, "温度上限", 40, 110, 80.0, "temp_limit", "{:.0f}")

        # 电气数据
        elecf = self._section(self.left, "电气数据")
        rkt = ttk.Frame(elecf); rkt.pack(fill=tk.X, pady=1)
        ttk.Label(rkt, text="KT(Nm/A)", width=10).pack(side=tk.LEFT)
        self.kt_var = tk.StringVar(value="1.20")
        kt_entry = ttk.Entry(rkt, textvariable=self.kt_var, width=7)
        kt_entry.pack(side=tk.LEFT, padx=3)
        kt_entry.bind("<Return>", lambda e: self._apply_kt())
        ttk.Button(rkt, text="应用", width=5, command=self._apply_kt).pack(side=tk.LEFT)
        ttk.Button(elecf, text="读取电气参数", command=lambda: self.worker.post("read_elec")).pack(fill=tk.X, pady=2)
        self.elec_cur_lbl = ttk.Label(elecf, text="估算电流 I = -- A", font=("Consolas", 10))
        self.elec_cur_lbl.pack(anchor=tk.W)
        self.elec_par_lbl = ttk.Label(elecf, text="欠压UV=-- 过压OV=--\n过流OC=-- KT读值=--",
                                      foreground="gray", justify=tk.LEFT)
        self.elec_par_lbl.pack(anchor=tk.W)
        ttk.Label(elecf, text="注: 反馈帧无实时电压/电流,\n电流由 I=τ/KT 估算, 电压仅阈值",
                  foreground="#888", font=("", 8), justify=tk.LEFT).pack(anchor=tk.W)

        # 记录
        recf = self._section(self.left, "数据记录")
        self.rec_var = tk.BooleanVar(value=False)
        ttk.Button(recf, text="开始/停止 CSV 记录", command=self._toggle_csv).pack(fill=tk.X)
        self.rec_lbl = ttk.Label(recf, text="未记录")
        self.rec_lbl.pack(anchor=tk.W)

        # 状态栏
        self.status_lbl = ttk.Label(self.left, text="状态: 未连接", relief=tk.SUNKEN, anchor=tk.W)
        self.status_lbl.pack(fill=tk.X, pady=(8, 0))

    def _apply_kt(self):
        try:
            kt = float(self.kt_var.get())
            if kt >= 0.1:
                self.worker.set_param("kt", kt)
        except ValueError:
            pass

    # ---------- 曲线 ----------
    def _build_plots(self):
        self.fig = Figure(figsize=(8, 7), dpi=96)
        self.fig.subplots_adjust(left=0.1, right=0.97, top=0.97, bottom=0.06, hspace=0.35)
        self.ax_pos = self.fig.add_subplot(411)
        self.ax_vel = self.fig.add_subplot(412, sharex=self.ax_pos)
        self.ax_tau = self.fig.add_subplot(413, sharex=self.ax_pos)
        self.ax_temp = self.fig.add_subplot(414, sharex=self.ax_pos)

        self.ax_pos.set_ylabel("开度 %")
        self.ax_vel.set_ylabel("速度 rad/s")
        self.ax_tau.set_ylabel("力矩 Nm")
        self.ax_temp.set_ylabel("温度 C")
        self.ax_temp.set_xlabel("时间 s")
        for ax in (self.ax_pos, self.ax_vel, self.ax_tau, self.ax_temp):
            ax.grid(True, alpha=0.3)

        (self.ln_pos,) = self.ax_pos.plot([], [], "b-", lw=1.2, label="开度%")
        (self.ln_vel,) = self.ax_vel.plot([], [], "g-", lw=1.0, label="vel")
        (self.ln_tau,) = self.ax_tau.plot([], [], "r-", lw=1.0, label="tau实测")
        (self.ln_taucmd,) = self.ax_tau.plot([], [], "m--", lw=0.9, label="tau指令")
        (self.ln_tmos,) = self.ax_temp.plot([], [], "-", color="orange", lw=1.0, label="Tmos")
        (self.ln_troto,) = self.ax_temp.plot([], [], "-", color="brown", lw=1.0, label="Troto")
        self.ax_pos.legend(loc="upper right", fontsize=8)
        self.ax_tau.legend(loc="upper right", fontsize=8)
        self.ax_temp.legend(loc="upper right", fontsize=8)
        self.ax_pos.set_ylim(-10, 110)
        # 力矩图右侧附加"估算电流 A"坐标轴 (I=tau/KT)
        self.ax_cur = None
        self._cur_kt = None
        # 保护限值参考线 (创建一次, 后续仅更新 y 值/可见性, 避免每帧 remove/add)
        self.lim_vel_hi = self.ax_vel.axhline(0, color="gray", ls=":", lw=0.8)
        self.lim_vel_lo = self.ax_vel.axhline(0, color="gray", ls=":", lw=0.8)
        self.lim_tau_hi = self.ax_tau.axhline(0, color="gray", ls=":", lw=0.8)
        self.lim_tau_lo = self.ax_tau.axhline(0, color="gray", ls=":", lw=0.8)
        self.lim_temp = self.ax_temp.axhline(0, color="red", ls=":", lw=0.8)
        self._prot_fill = None

        canvas_frame = ttk.Frame(self.right)
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        NavigationToolbar2Tk(self.canvas, canvas_frame)

        # 数值读数大字
        self.read_lbl = ttk.Label(self.right, text="", font=("Consolas", 11))
        self.read_lbl.pack(fill=tk.X, pady=2)

    # ---------- 事件 ----------
    def _toggle_conn(self):
        st = self.worker.status()
        if not st["connected"]:
            self.worker.post("connect", self.port_var.get(), DEFAULT_BAUD)
        else:
            self.worker.post("disconnect")

    def _toggle_enable(self):
        st = self.worker.status()
        if not st["enabled"]:
            self.worker.post("enable")
        else:
            self.worker.post("disable")

    def _on_mode(self):
        self.worker.set_param("mode", self.mode_var.get())

    def _do_zero(self):
        if not self.worker.status()["connected"]:
            messagebox.showwarning("设零位", "请先连接电机")
            return
        if messagebox.askyesno(
                "设零位 (谨慎)",
                "设零位会把当前位置设为新的 0 点, 导致整个行程坐标偏移,\n"
                "原有的全开/全合限位将失效!\n\n"
                "操作后请务必点击\"初始化自检\"重新标定, 恢复一致性.\n\n确认设零位?"):
            self.worker.post("zero")

    def _do_selfcheck(self):
        if not self.worker.status()["connected"]:
            messagebox.showwarning("初始化自检", "请先连接电机")
            return
        if messagebox.askyesno(
                "初始化自检",
                "将执行: 同步量程/电气参数 -> 使能 -> 缓慢顶到两端机械限位\n"
                "重新标定行程 -> 复位模式/目标/保护 -> 回到中点.\n\n"
                "用于误标零或状态错乱后恢复电机与软件一致性.\n"
                "请确保夹爪内无障碍物且可自由活动. 确认继续?"):
            self.worker.post("selfcheck")

    def _sync_slider_ranges(self):
        """重标定后, 将位置相关滑条范围与目标对齐到新的 OPEN/CLOSE。"""
        for s in (getattr(self, "s_goal", None), getattr(self, "s_mit_q", None)):
            if s is not None:
                try:
                    s.config(from_=CLOSE_END, to=OPEN_END)
                except Exception:
                    pass
        try:
            self.s_goal.set(self.worker.get_params()["q_goal"])
        except Exception:
            pass

    def _set_goal(self, v):
        self.worker.set_param("q_goal", v)
        try:
            self.s_goal.set(v)
        except Exception:
            pass

    def _toggle_csv(self):
        if not self.rec_var.get():
            fname = time.strftime("gripper_log_%Y%m%d_%H%M%S.csv")
            self.worker.post("csv_start", fname)
            self.rec_var.set(True)
            self.rec_lbl.config(text=f"记录中: {fname}")
        else:
            self.worker.post("csv_stop")
            self.rec_var.set(False)
            self.rec_lbl.config(text="未记录")

    # ---------- 刷新 ----------
    def _refresh_loop(self):
        if not self._alive:
            return
        try:
            st = self.worker.status()
            data = self.worker.snapshot()
            self._update_status(st)
            if data:
                self._update_plots(data, st)
            self._update_elec()
            lv = st.get("limits_version", 0)
            if lv != self._last_limits_version:
                self._last_limits_version = lv
                self._sync_slider_ranges()
            self.btn_en.config(state=(tk.NORMAL if st["connected"] else tk.DISABLED))
            self.btn_conn.config(text=("断开" if st["connected"] else "连接"))
            self.btn_en.config(text=("失能" if st["enabled"] else "使能"))
        except tk.TclError:
            return
        if self._alive:
            self._after_id = self.root.after(REFRESH_MS, self._refresh_loop)

    def _update_status(self, st):
        if not st["connected"]:
            txt, color = "状态: 未连接", "gray"
        elif st["calibrating"]:
            txt, color = "状态: 标定中...", "blue"
        elif st["prot_state"] == "FAULT":
            txt, color = f"故障: {st['prot_reason']}", "red"
        elif st["prot_state"] == "PROTECT":
            txt, color = f"保护: {st['prot_reason']}", "orange"
        elif st["enabled"]:
            txt, color = "状态: 运行中 (NORMAL)", "green"
        else:
            txt, color = "状态: 已连接, 未使能", "black"
        self.status_lbl.config(text=txt, foreground=color)

    def _update_plots(self, data, st):
        import numpy as np
        arr = np.array(data, dtype=float)
        t = arr[:, 0]
        tmax = t[-1]
        tmin = tmax - WINDOW_S
        m = t >= tmin
        t = t[m]
        pct, q, dq, tau, taucmd, tmos, troto, prot = (arr[m, i] for i in range(1, 9))

        self.ln_pos.set_data(t, pct)
        self.ln_vel.set_data(t, dq)
        self.ln_tau.set_data(t, tau)
        self.ln_taucmd.set_data(t, taucmd)
        self.ln_tmos.set_data(t, tmos)
        self.ln_troto.set_data(t, troto)

        self.ax_pos.set_xlim(tmin, tmax + 0.1)

        p = self.worker.get_params()
        # 自适应 y 轴
        self.ax_vel.set_ylim(-max(1.0, p["vel_limit"] * 1.2), max(1.0, p["vel_limit"] * 1.2))
        tl = max(0.5, p["tau_limit"] * 1.3)
        self.ax_tau.set_ylim(-tl, tl)
        self._ensure_cur_axis(p.get("kt", 1.2))
        if len(tmos):
            tc = max(tmos.max(), troto.max(), p["temp_limit"]) + 5
            self.ax_temp.set_ylim(0, tc)

        # 保护限值参考线 (原地更新 y 值与可见性)
        self.lim_vel_hi.set_ydata([p["vel_limit"], p["vel_limit"]])
        self.lim_vel_lo.set_ydata([-p["vel_limit"], -p["vel_limit"]])
        self.lim_vel_hi.set_visible(p["prot_vel_on"]); self.lim_vel_lo.set_visible(p["prot_vel_on"])
        self.lim_tau_hi.set_ydata([p["tau_limit"], p["tau_limit"]])
        self.lim_tau_lo.set_ydata([-p["tau_limit"], -p["tau_limit"]])
        self.lim_tau_hi.set_visible(p["prot_tau_on"]); self.lim_tau_lo.set_visible(p["prot_tau_on"])
        self.lim_temp.set_ydata([p["temp_limit"], p["temp_limit"]])
        self.lim_temp.set_visible(p["prot_temp_on"])

        # 保护区间标红 (在力矩图上, 每帧重建)
        if self._prot_fill is not None:
            try:
                self._prot_fill.remove()
            except Exception:
                pass
        self._prot_fill = self.ax_tau.fill_between(t, -tl, tl, where=(prot > 0.5),
                                                   color="red", alpha=0.12, step="mid")

        # 同步绘制: 下一次刷新在本次绘制完成后才排程(见 _refresh_loop),
        # 保证固定空闲间隔, 避免重绘耗时>刷新间隔时事件循环饱和
        self.canvas.draw()

        # 读数 (含估算电流)
        kt = max(0.1, p.get("kt", 1.2))
        cur = tau[-1] / kt
        self.elec_cur_lbl.config(text=f"估算电流 I = {cur:+.2f} A  (KT={kt:.2f})")
        self.read_lbl.config(text=(
            f"开度 {pct[-1]:5.1f}%  q={q[-1]:+.3f}rad  vel={dq[-1]:+.2f}rad/s  "
            f"tau={tau[-1]:+.2f}Nm  I≈{cur:+.2f}A  Tmos={tmos[-1]:.0f}C  Troto={troto[-1]:.0f}C  "
            f"[{st['prot_state']}]"))

    def _ensure_cur_axis(self, kt):
        kt = max(0.1, kt)
        if self.ax_cur is not None and self._cur_kt == kt:
            return
        if self.ax_cur is not None:
            try:
                self.ax_cur.remove()
            except Exception:
                pass
        self.ax_cur = self.ax_tau.secondary_yaxis(
            "right", functions=(lambda x, k=kt: x / k, lambda x, k=kt: x * k))
        self.ax_cur.set_ylabel("电流 A(估算)")
        self._cur_kt = kt

    def _update_elec(self):
        e = self.worker.get_elec()

        def f(v, u=""):
            return "--" if v is None else f"{v:.2f}{u}"
        self.elec_par_lbl.config(text=(
            f"欠压UV={f(e['UV'],'V')} 过压OV={f(e['OV'],'V')}\n"
            f"过流OC={f(e['OC'])} KT读值={f(e['KT'])}"))

    def _on_close(self):
        self._alive = False
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
        self.worker.stop_flag = True
        # 等后台线程安全收尾 (失能/关串口)
        self.worker.join(timeout=2.0)
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass
    GripperStudio(root)
    root.mainloop()


if __name__ == "__main__":
    main()
