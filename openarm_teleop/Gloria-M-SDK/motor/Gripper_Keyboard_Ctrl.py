import time
import sys
import serial
from CAN import *
from XuanYa_Comm import MotorComm, gloria_max

try:
    import msvcrt  # Windows 键盘非阻塞读取
except ImportError:
    msvcrt = None

"""
夹爪键盘实时控制 (行程控制 + 夹持力控制)
========================================
实测标定 (Gripper_Calib.py):
    OPEN  (全开) ≈ -0.31 rad      CLOSE (全合) ≈ -2.73 rad
    负方向 = 闭合, 正方向 = 张开

两种模式:
  [POSITION 行程模式]  MIT 位置控制 (kp/kd), 目标位置带限速平滑, 控制钳口开度。
  [FORCE 夹持力模式]   MIT 纯力矩 (kp=0)+速度阻尼, 给定夹持力(闭合方向)抓取并保持。

按键 (需在真实终端运行, 非 IDE 输出窗口):
  o : 张到全开        c : 合到全合        m : 移到中点
  a : 张开一点(+)     d : 闭合一点(-)     空格: 急停/保持当前位置
  p : 切到 行程模式   f : 切到 夹持力模式
  g : 夹持(施加当前目标力, 闭合)   r : 松开(力归零)
  + : 夹持力 +0.1Nm   - : 夹持力 -0.1Nm
  q / ESC : 退出 (失能电机)

运行 (motor/ 目录下):  python Gripper_Keyboard_Ctrl.py
"""

PORT = 'COM6'
BAUD = 921600

# ---- 实测端点 (留安全余量, 避免硬撞限位) ----
OPEN_END = -0.35
CLOSE_END = -2.70
MID = (OPEN_END + CLOSE_END) / 2.0

# ---- 行程(位置)模式增益 ----
KP_POS = 12.0
KD_POS = 1.0
POS_STEP = 0.08          # 'a'/'d' 单次微调步长 (rad)
MAX_SLEW = 0.012         # 每周期目标最大移动量 (rad), 限速平滑 -> ~2.4rad/s @200Hz

# ---- 夹持力模式 ----
KD_FORCE = 0.8           # 速度阻尼, 防空载失控
GRIP_STEP = 0.1          # 夹持力调节步长 (Nm)
GRIP_MIN, GRIP_MAX = 0.0, 0.8   # 本次测试限定小力档
GRIP_DEFAULT = 0.3

VEL_LIMIT = 8.0          # rad/s 安全切断
SEND_HZ = 200.0


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def read_key():
    """非阻塞读取一个按键, 无输入返回 None。"""
    if msvcrt is None:
        return None
    if msvcrt.kbhit():
        ch = msvcrt.getch()
        if ch in (b'\x00', b'\xe0'):  # 功能键/方向键前缀, 再读一字节丢弃
            msvcrt.getch()
            return None
        try:
            return ch.decode('ascii', 'ignore').lower()
        except Exception:
            return None
    return None


def goto(mc, motor, comm, q_goal, send_dt, settle=0.6, timeout=6.0, label=""):
    """限速平滑移动到目标位置 (与键盘工具同一控制路径), 用于无键盘自检。"""
    q_slew = comm.getPosRad(motor)
    t0 = time.time()
    settle_since = None
    while time.time() - t0 < timeout:
        q_cur = comm.getPosRad(motor)
        dq = comm.getVelRad(motor)
        if abs(dq) > VEL_LIMIT:
            print(f"\n[SAFETY] |vel|={abs(dq):.2f} 切断")
            break
        step = clamp(q_goal - q_slew, -MAX_SLEW, MAX_SLEW)
        q_slew += step
        mc.controlMIT(motor, KP_POS, KD_POS, q_slew, 0.0, 0.0)
        if abs(q_cur - q_goal) < 0.03 and abs(dq) < 0.05:
            if settle_since is None:
                settle_since = time.time()
            elif time.time() - settle_since >= settle:
                break
        else:
            settle_since = None
        time.sleep(send_dt)
    q_cur = comm.getPosRad(motor)
    print(f"  [{label}] 到位 q={q_cur:6.3f}rad ({comm.getAngle100(motor):5.1f}%) tau={comm.getTorque(motor):5.2f}Nm")


def selftest(mc, motor, comm, send_dt):
    print("=== 行程控制自检 (OPEN -> CLOSE -> MID) ===")
    goto(mc, motor, comm, OPEN_END, send_dt, label="OPEN ")
    time.sleep(0.3)
    goto(mc, motor, comm, CLOSE_END, send_dt, label="CLOSE")
    time.sleep(0.3)
    goto(mc, motor, comm, MID, send_dt, label="MID  ")


def main():
    self_test = '--selftest' in sys.argv
    if msvcrt is None and not self_test:
        print("当前平台无 msvcrt, 键盘实时控制仅支持 Windows 终端。")
        return

    motor = Motor(DM_Motor_Type.DM4310, 0x01, 0x101)
    serial_device = serial.Serial(PORT, BAUD, timeout=0.5)
    mc = MotorControl(serial_device)
    mc.addMotor(motor)
    comm = MotorComm()

    # 同步实际量程
    pmax = mc.read_motor_param(motor, DM_variable.PMAX)
    vmax = mc.read_motor_param(motor, DM_variable.VMAX)
    tmax = mc.read_motor_param(motor, DM_variable.TMAX)
    if pmax and vmax and tmax:
        mc.change_limit_param(DM_Motor_Type.DM4310, pmax, vmax, tmax)

    mc.disable(motor)
    time.sleep(0.5)
    mc.enable(motor)
    time.sleep(0.2)
    mc.refresh_motor_status(motor)

    send_dt0 = 1.0 / SEND_HZ
    if self_test:
        try:
            selftest(mc, motor, comm, send_dt0)
        finally:
            for _ in range(20):
                mc.controlMIT(motor, 0.0, KD_FORCE, 0.0, 0.0, 0.0)
                time.sleep(send_dt0)
            mc.disable(motor)
            serial_device.close()
            print("自检结束, 电机已失能。")
        return

    q_cur = comm.getPosRad(motor)
    mode = "POSITION"          # POSITION / FORCE
    q_goal = clamp(q_cur, CLOSE_END, OPEN_END)
    q_slew = q_cur
    grip_force = GRIP_DEFAULT
    gripping = False           # FORCE 模式下是否正在施力
    send_dt = 1.0 / SEND_HZ
    last_print = 0.0
    running = True

    print(__doc__)
    print(f"[READY] 起始 q={q_cur:.3f}rad  模式=POSITION  夹持力={grip_force:.1f}Nm")

    try:
        while running:
            now = time.time()
            q_cur = comm.getPosRad(motor)
            dq = comm.getVelRad(motor)
            tau_fb = comm.getTorque(motor)

            # ---------- 按键处理 ----------
            k = read_key()
            if k is not None:
                if k in ('q', '\x1b'):
                    running = False
                elif k == 'p':
                    mode = "POSITION"; q_goal = clamp(q_cur, CLOSE_END, OPEN_END); q_slew = q_cur
                elif k == 'f':
                    mode = "FORCE"; gripping = False
                elif k == 'o':
                    q_goal = OPEN_END
                elif k == 'c':
                    q_goal = CLOSE_END
                elif k == 'm':
                    q_goal = MID
                elif k == 'a':       # 张开一点 (正方向)
                    q_goal = clamp(q_goal + POS_STEP, CLOSE_END, OPEN_END)
                elif k == 'd':       # 闭合一点 (负方向)
                    q_goal = clamp(q_goal - POS_STEP, CLOSE_END, OPEN_END)
                elif k == ' ':
                    if mode == "POSITION":
                        q_goal = q_cur; q_slew = q_cur
                    else:
                        gripping = False
                elif k == 'g':
                    gripping = True; mode = "FORCE"
                elif k == 'r':
                    gripping = False
                elif k in ('+', '='):
                    grip_force = clamp(grip_force + GRIP_STEP, GRIP_MIN, GRIP_MAX)
                elif k == '-':
                    grip_force = clamp(grip_force - GRIP_STEP, GRIP_MIN, GRIP_MAX)

            # ---------- 速度安全 ----------
            if abs(dq) > VEL_LIMIT:
                print(f"\n[SAFETY] |vel|={abs(dq):.2f} > {VEL_LIMIT}, 切换保持")
                mode = "POSITION"; q_goal = q_cur; q_slew = q_cur; gripping = False

            # ---------- 控制输出 ----------
            if mode == "POSITION":
                # 目标限速平滑
                step = clamp(q_goal - q_slew, -MAX_SLEW, MAX_SLEW)
                q_slew += step
                mc.controlMIT(motor, KP_POS, KD_POS, q_slew, 0.0, 0.0)
            else:  # FORCE
                # 夹持 = 闭合方向 = 负力矩
                tau_cmd = -grip_force if gripping else 0.0
                mc.controlMIT(motor, 0.0, KD_FORCE, 0.0, 0.0, tau_cmd)

            # ---------- 状态打印 ----------
            if now - last_print >= 0.1:
                last_print = now
                pct = comm.getAngle100(motor)
                if mode == "POSITION":
                    info = f"goal={q_goal:6.3f}"
                else:
                    info = f"grip={'ON ' if gripping else 'OFF'} F*={grip_force:.1f}Nm"
                sys.stdout.write(
                    f"\r[{mode:8s}] q={q_cur:6.3f}rad ({pct:5.1f}%) vel={dq:6.2f} tau={tau_fb:6.2f}Nm "
                    f"{info} Tmos={comm.getTMos(motor)}C   ")
                sys.stdout.flush()

            time.sleep(send_dt)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(20):
            mc.controlMIT(motor, 0.0, KD_FORCE, 0.0, 0.0, 0.0)
            time.sleep(send_dt)
        mc.disable(motor)
        serial_device.close()
        print("\n退出, 电机已失能, 串口已关闭。")


if __name__ == "__main__":
    main()
