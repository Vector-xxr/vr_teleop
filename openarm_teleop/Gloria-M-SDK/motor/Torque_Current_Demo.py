import time
import serial
from CAN import *
from XuanYa_Comm import MotorComm, gloria_max

"""
电流 / 力矩控制调试 Demo
------------------------
达妙 DM4310 在 MIT 模式下: 电调把解算出的扭矩值作为"电流环的电流给定",
因此【力矩控制 == 电流控制】(电流 ≈ 力矩 / KT)。

本 Demo 用 MIT 模式做纯力矩输出 (kp=0)，并叠加一个很小的速度阻尼 kd
作为【失控保护】(空载时纯力矩会让夹爪持续加速)。流程:
    0 -> 目标力矩 (ramp 上升) -> 保持 -> 回到 0 -> 失能
全程监测速度，超过 vel_limit 立即切断力矩。

注意 (来自 doc/ 说明书):
    DM4310 额定扭矩 3Nm / 峰值 7Nm, 额定电流 2.5A / 峰值 7.5A。
    调试请从 0.2~0.5Nm 小力矩起步, 确认方向与发热后再增大。

运行 (在 motor/ 目录下):
    python Torque_Current_Demo.py
Ctrl+C 可随时安全退出 (失能电机)。
"""

# ---------------- 可调参数 ----------------
PORT = 'COM6'
BAUD = 921600
TAU_TARGET = 0.3          # 目标力矩 (Nm) —— 保守起步值
KD_SAFETY = 0.8           # 速度阻尼, 防止空载力矩失控 (纯力矩控制可设 0, 但不推荐空载)
VEL_LIMIT = 8.0           # 速度安全阈值 (rad/s)，超过立即切断
RAMP_UP_T = 1.5           # 力矩爬升时间 (s)
HOLD_T = 2.0              # 保持时间 (s)
RAMP_DOWN_T = 1.0         # 力矩回落时间 (s)
SEND_HZ = 200.0
PRINT_HZ = 20.0


def sync_limits(mc, motor):
    """读取电机内实际 PMAX/VMAX/TMAX 并同步到 SDK 的 Limit_Param,
    修正 SDK 默认 VMAX(20) 与电机配置(10) 不一致导致的速度量纲错误。"""
    pmax = mc.read_motor_param(motor, DM_variable.PMAX)
    vmax = mc.read_motor_param(motor, DM_variable.VMAX)
    tmax = mc.read_motor_param(motor, DM_variable.TMAX)
    kt = mc.read_motor_param(motor, DM_variable.KT_Value)
    if pmax is not None and vmax is not None and tmax is not None:
        mc.change_limit_param(DM_Motor_Type.DM4310, pmax, vmax, tmax)
        print(f"[SYNC] Limit_Param <- PMAX={pmax:.3f} VMAX={vmax:.3f} TMAX={tmax:.3f}")
    print(f"[INFO] KT_Value={kt}  (KT=0 表示未标定, 电流=力矩/KT 不可用, 以力矩反馈为准)")
    return kt


def est_current(tau, kt):
    """由力矩反馈估算相电流 (A)。KT 未标定(=0)时返回 None。"""
    if kt and kt > 1e-6:
        return tau / kt
    return None


def main():
    motor = Motor(DM_Motor_Type.DM4310, 0x01, 0x101)
    serial_device = serial.Serial(PORT, BAUD, timeout=0.5)
    mc = MotorControl(serial_device)
    mc.addMotor(motor)
    comm = MotorComm()

    kt = sync_limits(mc, motor)

    mc.disable(motor)
    time.sleep(0.5)
    mc.enable(motor)
    time.sleep(0.2)

    send_dt = 1.0 / SEND_HZ
    print_dt = 1.0 / PRINT_HZ
    last_print = 0.0
    total_t = RAMP_UP_T + HOLD_T + RAMP_DOWN_T

    print(f"=== Torque/Current Control Demo ===  target={TAU_TARGET}Nm kd_safety={KD_SAFETY} vel_limit={VEL_LIMIT}rad/s")
    t0 = time.time()
    aborted = False
    try:
        while True:
            now = time.time()
            t = now - t0
            if t >= total_t:
                break

            # 力矩梯形曲线: 上升 -> 保持 -> 回落
            if t < RAMP_UP_T:
                tau_cmd = TAU_TARGET * (t / RAMP_UP_T)
                phase = "RAMP_UP"
            elif t < RAMP_UP_T + HOLD_T:
                tau_cmd = TAU_TARGET
                phase = "HOLD"
            else:
                td = t - RAMP_UP_T - HOLD_T
                tau_cmd = TAU_TARGET * max(0.0, 1.0 - td / RAMP_DOWN_T)
                phase = "RAMP_DOWN"

            # 读取反馈
            dq = comm.getVelRad(motor)
            # 速度安全保护: 空载力矩失控时立即切断
            if abs(dq) > VEL_LIMIT:
                print(f"[SAFETY] |vel|={abs(dq):.2f}rad/s > {VEL_LIMIT}, 切断力矩!")
                tau_cmd = 0.0
                aborted = True

            # 纯力矩 (kp=0) + 速度阻尼 (kd) 输出
            mc.controlMIT(motor, 0.0, KD_SAFETY, 0.0, 0.0, tau_cmd)

            if now - last_print >= print_dt:
                last_print = now
                q = comm.getPosRad(motor)
                tau_fb = comm.getTorque(motor)
                cur = est_current(tau_fb, kt)
                cur_s = f"{cur:.2f}A" if cur is not None else "N/A"
                print(f"[{phase:9s}] t={t:4.2f}s tauCmd={tau_cmd:5.2f}Nm tauFb={tau_fb:6.2f}Nm "
                      f"cur={cur_s} pos={q:6.3f}rad vel={dq:6.2f}rad/s "
                      f"Tmos={comm.getTMos(motor)}C Troto={comm.getTRoto(motor)}C")

            if aborted:
                break
            time.sleep(send_dt)
    except KeyboardInterrupt:
        print("\n[CTRL-C] 用户中断")
    finally:
        # 安全收尾: 力矩归零并失能
        for _ in range(20):
            mc.controlMIT(motor, 0.0, KD_SAFETY, 0.0, 0.0, 0.0)
            time.sleep(send_dt)
        mc.disable(motor)
        serial_device.close()
        print("Demo 结束, 电机已失能, 串口已关闭。")


if __name__ == "__main__":
    main()
