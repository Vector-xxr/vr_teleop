import serial
from CAN import *


def main():
    Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x101)
    serial_device = serial.Serial('COM6', 921600, timeout=0.5)
    mc = MotorControl(serial_device)
    mc.addMotor(Motor1)

    print("=== Connectivity / Param Read Test (read-only) ===")
    items = [
        ("sub_ver", DM_variable.sub_ver),
        ("CTRL_MODE", DM_variable.CTRL_MODE),
        ("Gr", DM_variable.Gr),
        ("PMAX", DM_variable.PMAX),
        ("VMAX", DM_variable.VMAX),
        ("TMAX", DM_variable.TMAX),
        ("KT_Value", DM_variable.KT_Value),
        ("OC_Value", DM_variable.OC_Value),
        ("MST_ID", DM_variable.MST_ID),
        ("ESC_ID", DM_variable.ESC_ID),
    ]
    ok = 0
    for name, rid in items:
        val = mc.read_motor_param(Motor1, rid)
        print(f"{name:10s} (RID {int(rid):2d}) = {val}")
        if val is not None:
            ok += 1
    print(f"--- read {ok}/{len(items)} params successfully ---")

    serial_device.close()


if __name__ == "__main__":
    main()
