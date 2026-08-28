import time
import serial
from CAN import *
from XuanYa_Comm import MotorComm, gloria_max

"""
夹爪行程标定 / 试探 (低增益、空载安全版)
----------------------------------------
用很小的力矩 + 速度阻尼, 分别向正/负方向缓慢驱动到机械限位 (堵转),
记录两端位置, 从而得到夹爪真实行程范围与"全开/全合"方向。
全程速度保护, 到位即停, 不会硬冲。

运行 (motor/ 目录下): python Gripper_Calib.py
"""

PORT = 'COM6'
BAUD = 921600
PROBE_TAU = 0.5         # 试探力矩 (Nm), 需高于约 0.3Nm 的起步摩擦力才能顶到限位
KD = 0.9                # 速度阻尼, 防止冲击
VEL_STALL = 0.05        # rad/s, 低于此视为"堵转/到位"
STALL_HOLD = 0.5        # s, 需连续堵转这么久才确认到位
MIN_DRIVE = 1.2         # s, 每个方向至少驱动这么久再判定 (兼容起步即在限位)
DIR_TIMEOUT = 12.0      # s, 单方向最长试探时间
VEL_LIMIT = 6.0         # rad/s, 安全切断
SEND_HZ = 200.0


def probe(mc, motor, comm, sign, label):
    send_dt = 1.0 / SEND_HZ
    t0 = time.time()
    stall_since = None
    last_print = 0.0
    q = comm.getPosRad(motor)
    while time.time() - t0 < DIR_TIMEOUT:
        now = time.time()
        mc.controlMIT(motor, 0.0, KD, 0.0, 0.0, sign * PROBE_TAU)
        dq = comm.getVelRad(motor)
        q = comm.getPosRad(motor)
        if abs(dq) > VEL_LIMIT:
            print(f"[SAFETY] |vel|={abs(dq):.2f} > {VEL_LIMIT}, 切断")
            break
        if abs(dq) < VEL_STALL:
            if stall_since is None:
                stall_since = now
            elif (now - stall_since) >= STALL_HOLD and (now - t0) >= MIN_DRIVE:
                break
        else:
            stall_since = None
        if now - last_print >= 0.2:
            last_print = now
            print(f"  [{label}] q={q:6.3f}rad vel={dq:6.2f}rad/s tau={comm.getTorque(motor):5.2f}Nm Tmos={comm.getTMos(motor)}C")
        time.sleep(send_dt)
    return q


def main():
    motor = Motor(DM_Motor_Type.DM4310, 0x01, 0x101)
    serial_device = serial.Serial(PORT, BAUD, timeout=0.5)
    mc = MotorControl(serial_device)
    mc.addMotor(motor)
    comm = MotorComm()

    # 同步实际量程, 修正速度量纲
    pmax = mc.read_motor_param(motor, DM_variable.PMAX)
    vmax = mc.read_motor_param(motor, DM_variable.VMAX)
    tmax = mc.read_motor_param(motor, DM_variable.TMAX)
    if pmax and vmax and tmax:
        mc.change_limit_param(DM_Motor_Type.DM4310, pmax, vmax, tmax)
        print(f"[SYNC] PMAX={pmax:.3f} VMAX={vmax:.3f} TMAX={tmax:.3f}")

    mc.disable(motor)
    time.sleep(0.5)
    mc.enable(motor)
    time.sleep(0.2)
    mc.refresh_motor_status(motor)
    q_start = comm.getPosRad(motor)
    print(f"[START] 起始位置 q={q_start:.3f}rad,  gloria_max={gloria_max}")

    try:
        print(">>> 向负方向 (-) 试探...")
        q_neg = probe(mc, motor, comm, -1.0, "NEG")
        time.sleep(0.3)
        print(">>> 向正方向 (+) 试探...")
        q_pos = probe(mc, motor, comm, +1.0, "POS")

        lo = min(q_neg, q_pos)
        hi = max(q_neg, q_pos)
        span = hi - lo
        print("\n================ 标定结果 ================")
        print(f"负向端: {q_neg:.3f} rad")
        print(f"正向端: {q_pos:.3f} rad")
        print(f"行程范围: [{lo:.3f}, {hi:.3f}] rad, 跨度 {span:.3f} rad ({span*180/3.14159:.1f} deg)")
        print(f"建议: OPEN/CLOSE 两端用上述值; 中点 {(lo+hi)/2:.3f} rad")
        print("==========================================")
    except KeyboardInterrupt:
        print("\n[CTRL-C] 中断")
    finally:
        for _ in range(20):
            mc.controlMIT(motor, 0.0, KD, 0.0, 0.0, 0.0)
            time.sleep(1.0 / SEND_HZ)
        mc.disable(motor)
        serial_device.close()
        print("标定结束, 电机已失能。")


if __name__ == "__main__":
    main()
