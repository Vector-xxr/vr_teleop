# -*- coding: utf-8 -*-
"""
Motor_Param_Tool.py —— 达妙 DM-J4310 电机参数配置工具
================================================================

定位（与 Gripper_Studio.py 的区别）
--------------------------------------------------------------
- Gripper_Studio.py：运行期“控制 + 实时曲线 + 保护调试”的 GUI 工作台。
- 本工具：专注“电机内部寄存器参数”的【读取 / 设置 / 保存到 Flash / 切换控制模式】，
  即面向电机本体的离线配置（调参接口 UART@921600）。两者职责不重叠。

参考文档：doc/dm4310-...pdf（特征参数 / 控制与保护章节）
寄存器定义：CAN.py 中 DM_variable 枚举（RID）。

参数写入流程（SDK）：
  change_motor_param() 通过 0x7FF 写寄存器，回读确认；
  save_motor_param()   先 disable 再 0xAA 写 Flash（断电不丢失）；
  switchControlMode()  写 RID=10 切换 MIT/POS_VEL/VEL/Torque_Pos。

用法
--------------------------------------------------------------
  # 图形界面（默认，直接运行即可，串口/端口在窗口内填写连接）
  python Motor_Param_Tool.py

  # 文本交互菜单
  python Motor_Param_Tool.py repl --port COM6

  # 读取全部参数并分类打印
  python Motor_Param_Tool.py --port COM6 dump
  python Motor_Param_Tool.py dump --all        # 含工厂校准寄存器

  # 读取 / 设置单个参数（名称或 RID 均可）
  python Motor_Param_Tool.py get PMAX
  python Motor_Param_Tool.py set ACC 5
  python Motor_Param_Tool.py set OC_Value 8.0 --save        # 写完顺带保存
  python Motor_Param_Tool.py set CTRL_MODE MIT              # 等价于 mode MIT

  # 切换控制模式 / 保存 / 导出
  python Motor_Param_Tool.py mode POS_VEL
  python Motor_Param_Tool.py save
  python Motor_Param_Tool.py export params.csv

注意：保护阈值 / 运动上限 / 通讯 ID 等属敏感参数，CLI 写入这些类别需加 --force，
交互菜单则会二次确认。
"""

import argparse
import csv
import os
import sys
import time

import serial

# 允许从仓库根目录或 motor/ 目录直接运行
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from CAN import (  # noqa: E402
    Motor,
    MotorControl,
    DM_Motor_Type,
    DM_variable,
    Control_Type,
    is_in_ranges,
)

# --------------------------------------------------------------------------
# 出厂/铭牌规格（来自说明书“特征参数 / 控制与保护”），用于写入前安全提示
# --------------------------------------------------------------------------
SPECS = {
    "额定电压": "24 V",
    "额定电流": "2.5 A",
    "峰值电流": "7.5 A",
    "额定扭矩": "3 Nm",
    "峰值扭矩": "7 Nm",
    "额定转速": "120 rpm",
    "空载最大转速": "200 rpm",
    "减速比": "10:1",
    "极对数": "14",
    "相电感": "340 uH",
    "相电阻": "650 mΩ",
}

# 说明书建议的保护边界（用于越界提醒，不是硬性拦截）
SAFE_HINT = {
    "OV_Value": ("过压阈值", "建议 ≤ 32 V"),
    "UV_Value": ("欠压阈值", "建议 ≥ 15 V"),
    "OC_Value": ("过流阈值", "标幺比例 0~1.0（触发电流/最大电流），不是安培！"),
    "OT_Value": ("过温阈值", "驱动 ≤ 120℃，电机建议 ≤ 100℃"),
}

# 读写级别：
#   RO   工厂/特征/版本，原则只读，改动可能损坏标定
#   RW   常规可调（运动整定、控制环）
#   LIMIT 运动上限，影响量程与映射，谨慎
#   PROT 保护阈值，关系安全
#   COMM 通讯/ID/模式，改后可能失联或改变行为
RO, RW, LIMIT, PROT, COMM = "RO", "RW", "LIMIT", "PROT", "COMM"
SENSITIVE = (LIMIT, PROT, COMM)

# --------------------------------------------------------------------------
# 寄存器表：(枚举, 分组, 单位, 级别, 说明)
# --------------------------------------------------------------------------
PARAMS = [
    # —— 设备信息 / 版本（只读）——
    (DM_variable.SN,        "设备信息", "",      RO, "序列号"),
    (DM_variable.hw_ver,    "设备信息", "",      RO, "硬件版本"),
    (DM_variable.sw_ver,    "设备信息", "",      RO, "软件版本"),
    (DM_variable.sub_ver,   "设备信息", "",      RO, "子版本号"),

    # —— 电机特征值（出厂标定，只读）——
    (DM_variable.KT_Value,  "电机特征", "Nm/A",  RO, "力矩常数 KT (I=τ/KT)"),
    (DM_variable.NPP,       "电机特征", "",      RO, "极对数 (出厂 14)"),
    (DM_variable.Gr,        "电机特征", "",      RO, "减速比 (出厂 10)"),
    (DM_variable.Rs,        "电机特征", "Ω",     RO, "相电阻 (出厂 ~0.65)"),
    (DM_variable.LS,        "电机特征", "H",     RO, "相电感 (出厂 ~340uH)"),
    (DM_variable.Flux,      "电机特征", "Wb",    RO, "磁链"),
    (DM_variable.Inertia,   "电机特征", "kg·m²", RO, "转动惯量(只读)"),

    # —— 运动上限（决定量程/编码映射，谨慎）——
    (DM_variable.PMAX,      "运动上限", "rad",   LIMIT, "位置上限 (±PMAX)"),
    (DM_variable.VMAX,      "运动上限", "rad/s", LIMIT, "速度上限 (±VMAX)"),
    (DM_variable.TMAX,      "运动上限", "Nm",    LIMIT, "力矩上限 (±TMAX)"),

    # —— 运动整定（常规可调）——
    (DM_variable.MAX_SPD,   "运动整定", "rad/s", RW, "位置模式最大速度"),
    (DM_variable.ACC,       "运动整定", "",      RW, "加速度（梯形, >0）"),
    (DM_variable.DEC,       "运动整定", "",      RW, "减速度（梯形, 取负值<0）"),
    (DM_variable.Damp,      "电机特征", "",      RO, "电机粘滞系数(只读)"),

    # —— 保护阈值（关系安全）——
    (DM_variable.UV_Value,  "保护阈值", "V",     PROT, "欠压保护(>10), 建议 ≥15V"),
    (DM_variable.OV_Value,  "保护阈值", "V",     PROT, "过压保护, 建议 ≤32V"),
    (DM_variable.OC_Value,  "保护阈值", "标幺",  PROT, "过流保护(0~1.0标幺比例), 非安培!"),
    (DM_variable.OT_Value,  "保护阈值", "℃",     PROT, "过温保护 [80,200), 电机建议 ≤100℃"),

    # —— 控制环参数（调参用）——
    (DM_variable.I_BW,      "控制环",   "Hz",    RW, "电流环控制带宽 [100,10000]"),
    (DM_variable.V_BW,      "控制环",   "Hz",    RW, "速度环滤波带宽 (0,500)"),
    (DM_variable.KP_ASR,    "控制环",   "",      RW, "速度环 Kp (≥0)"),
    (DM_variable.KI_ASR,    "控制环",   "",      RW, "速度环 Ki (≥0)"),
    (DM_variable.KP_APR,    "控制环",   "",      RW, "位置环 Kp (≥0)"),
    (DM_variable.KI_APR,    "控制环",   "",      RW, "位置环 Ki (≥0)"),
    (DM_variable.GREF,      "控制环",   "",      RW, "齿轮力矩效率 (0,1]"),
    (DM_variable.Deta,      "控制环",   "",      RW, "速度环阻尼系数 [1,30]"),
    (DM_variable.IQ_c1,     "控制环",   "",      RW, "电流环增强系数 [100,10000]"),
    (DM_variable.VL_c1,     "控制环",   "",      RW, "速度环增强系数 (0,10000]"),

    # —— 通讯 / ID / 模式（改后可能失联）——
    (DM_variable.ESC_ID,    "通讯/模式", "",     COMM, "电机 CAN ID (SlaveID)"),
    (DM_variable.MST_ID,    "通讯/模式", "",     COMM, "主机反馈 ID (MasterID)"),
    (DM_variable.CTRL_MODE, "通讯/模式", "",     COMM, "控制模式 1MIT/2POS_VEL/3VEL/4Torque_Pos"),
    (DM_variable.TIMEOUT,   "通讯/模式", "",     COMM, "通讯丢失超时（周期数，0=关闭）"),
    (DM_variable.can_br,    "通讯/模式", "",     COMM, "CAN 波特率档位（出厂 1Mbps，慎改）"),

    # —— 工厂校准（默认隐藏，--all 才读，强烈建议只读）——
    (DM_variable.u_off,     "工厂校准", "",      RO, "U 相零偏（标定）"),
    (DM_variable.v_off,     "工厂校准", "",      RO, "V 相零偏（标定）"),
    (DM_variable.k1,        "工厂校准", "",      RO, "编码器补偿 k1"),
    (DM_variable.k2,        "工厂校准", "",      RO, "编码器补偿 k2"),
    (DM_variable.m_off,     "工厂校准", "",      RO, "机械零偏"),
    (DM_variable.dir,       "工厂校准", "",      RO, "方向标志"),
    (DM_variable.p_m,       "工厂校准", "",      RO, "电角度/位置标定"),
    (DM_variable.xout,      "工厂校准", "",      RO, "输出标定"),
]

# --------------------------------------------------------------------------
# 详细说明（鼠标悬停“说明”列时弹出）：单位 / 出厂值 / 作用 / 注意事项
# --------------------------------------------------------------------------
DETAILS = {
    # 设备信息
    DM_variable.SN:       "序列号 (SN)\n出厂唯一标识，只读，用于区分设备。不可修改。",
    DM_variable.hw_ver:   "硬件版本号\n只读，标识驱动板硬件批次。",
    DM_variable.sw_ver:   "固件软件版本号\n只读。不同固件的使能/参数行为可能不同"
                          "（如旧固件 enable 需带控制模式偏移 enable_old）。",
    DM_variable.sub_ver:  "固件子版本号\n只读，固件修订细分号。",
    # 电机特征
    DM_variable.KT_Value: "力矩常数 KT (Nm/A)\n相电流与输出力矩的比例：I ≈ τ / KT。\n"
                          "出厂可能为 0（未标定），标定后才能由力矩反推电流。",
    DM_variable.NPP:      "极对数\nDM4310 出厂 = 14。只读，电角度换算依据，误改导致换向错误。",
    DM_variable.Gr:       "减速比\nDM4310 出厂 = 10:1。输出端力矩/速度 = 电机端 × Gr。",
    DM_variable.Rs:       "定子相电阻 Rs (Ω)\n出厂 ≈ 0.65。FOC 电流环模型参数，工厂标定，原则不改。",
    DM_variable.LS:       "定子相电感 Ls (H)\n出厂 ≈ 340µH。影响电流环带宽，工厂标定。",
    DM_variable.Flux:     "转子磁链 (Wb)\n转矩-电流模型参数，工厂标定，影响力矩精度。",
    DM_variable.Inertia:  "转动惯量 (kg·m²)\n【只读 RO】电机辨识/出厂值，用于速度环整定与前馈，不可写。",
    # 运动上限
    DM_variable.PMAX:     "位置量程上限 (rad)\n控制/反馈位置按 ±PMAX 线性映射到 16bit。\n"
                          "改后所有位置数值含义随之缩放，必须与上位机 Limit_Param 一致。",
    DM_variable.VMAX:     "速度量程上限 (rad/s)\n按 ±VMAX 映射 12bit。\n"
                          "注意：SDK 默认 20 与电机实际(常为10)若不一致，速度数值会差 2×。",
    DM_variable.TMAX:     "力矩量程上限 (Nm)\n按 ±TMAX 映射 12bit，出厂默认 12。\n"
                          "但 DM4310 额定扭矩仅 3Nm/峰值 7Nm，给定力矩务必远低于额定，从小值起步。",
    # 运动整定
    DM_variable.MAX_SPD:  "最大速度限制\n位置(梯形)模式下规划的最高运行速度。",
    DM_variable.ACC:      "加速度（梯形规划）\n值越大启动越快，但冲击与电流尖峰越大。",
    DM_variable.DEC:      "减速度（梯形规划）\n值越大停止越急。",
    DM_variable.Damp:     "电机粘滞系数\n【只读 RO】电机特征量，由固件辨识，不可写。\n"
                          "若要调阻尼请用控制指令里的 kd（MIT 模式）或速度环阻尼 Deta(RID31)。",
    # 保护阈值
    DM_variable.UV_Value: "欠压保护阈值 (V)\n母线电压低于此值将退出使能。建议 ≥ 15V。",
    DM_variable.OV_Value: "过压保护阈值 (V)\n母线电压高于此值将退出使能。建议 ≤ 32V，"
                          "防止急停/再生制动回馈过压损坏驱动。",
    DM_variable.OC_Value: "过流保护阈值（标幺比例，范围 0~1.0）\n"
                          "表示『触发电流 / 电机最大电流』的比例，不是安培数！\n"
                          "例如 0.8 ≈ 触发于 80% 最大电流。务必填 0~1 之间，"
                          "填 >1（如 8）会被电机直接拒绝写入。",
    DM_variable.OT_Value: "过温保护阈值 (℃)\n温度超过将退出使能。驱动 ≤ 120℃，"
                          "电机线圈建议 ≤ 100℃，避免退磁/烧毁。",
    # 控制环
    DM_variable.I_BW:     "电流环控制带宽 (Hz)\n范围 [100, 10000]。越高响应越快，"
                          "但过高易啸叫/不稳，受 Rs/Ls 限制。可写。",
    DM_variable.V_BW:     "速度环滤波带宽 (Hz)\n范围 (0, 500)。过高放大噪声。可写。",
    DM_variable.KP_ASR:   "速度环比例增益 Kp (ASR)\n范围 ≥0。提高跟随刚度，过大震荡。可写。",
    DM_variable.KI_ASR:   "速度环积分增益 Ki (ASR)\n范围 ≥0。消除稳态误差，过大超调/积分饱和。可写。",
    DM_variable.KP_APR:   "位置环比例增益 Kp (APR)\n范围 ≥0。提高定位刚度。可写。",
    DM_variable.KI_APR:   "位置环积分增益 Ki (APR)\n范围 ≥0。消除定位稳态误差。可写。",
    DM_variable.GREF:     "齿轮力矩效率\n范围 (0, 1.0]。减速箱效率系数，影响输出力矩换算。可写。",
    DM_variable.Deta:     "速度环阻尼系数\n范围 [1.0, 30.0]。增大更稳但响应变慢。可写。",
    DM_variable.IQ_c1:    "电流环增强系数\n范围 [100, 10000]。可写。",
    DM_variable.VL_c1:    "速度环增强系数\n范围 (0, 10000]。可写。",
    # 通讯/模式
    DM_variable.ESC_ID:   "电机 CAN ID (SlaveID)\n控制帧寻址用。改后须『保存』并重新上电，"
                          "且用新 ID 重新连接，否则失联。多电机时各机必须唯一。",
    DM_variable.MST_ID:   "主机反馈 ID (MasterID)\n电机回传反馈帧所用 ID。多电机时需各异以区分。",
    DM_variable.CTRL_MODE:"控制模式\n1=MIT，2=位置速度，3=速度，4=力位混合(Torque_Pos)。\n"
                          "决定使能后接收的控制帧格式；切换建议用『应用模式』。",
    DM_variable.TIMEOUT:  "通讯丢失超时（周期数）\n设定周期内未收到 CAN 指令则自动失能；0=关闭该保护。",
    DM_variable.can_br:   "CAN 波特率档位\n出厂 1Mbps。改动后整条总线通信参数变化，"
                          "需同步修改 USB-CAN 适配器，否则失联，强烈建议保持默认。",
    # 工厂校准（只读）
    DM_variable.u_off:    "U 相电流采样零偏\n工厂标定，改动会破坏 FOC 电流环，原则只读。",
    DM_variable.v_off:    "V 相电流采样零偏\n工厂标定，原则只读。",
    DM_variable.k1:       "编码器线性化补偿系数 k1\n工厂标定，原则只读。",
    DM_variable.k2:       "编码器线性化补偿系数 k2\n工厂标定，原则只读。",
    DM_variable.m_off:    "机械/电角度零位偏移\n工厂标定，误改导致定位与换向错乱，原则只读。",
    DM_variable.dir:      "电机方向标志\n工厂标定，原则只读。",
    DM_variable.p_m:      "位置/电角度标定相关\n工厂参数，原则只读。",
    DM_variable.xout:     "输出标定相关\n工厂参数，原则只读。",
}

# --------------------------------------------------------------------------
# 合法取值范围 (lo, hi)，None 表示该侧不限。来源：DM 官方寄存器表。
# 写入前用于校验/提示——超范围电机会拒绝写入（“设了没反应”的常见原因）。
# --------------------------------------------------------------------------
RANGE = {
    DM_variable.UV_Value:  (10.0, None),
    DM_variable.OT_Value:  (80.0, 200.0),
    DM_variable.OC_Value:  (0.0, 1.0),       # 标幺比例, 非安培
    DM_variable.OV_Value:  (10.0, None),
    DM_variable.ACC:       (0.0, None),
    DM_variable.DEC:       (None, 0.0),      # 减速度取负
    DM_variable.MAX_SPD:   (0.0, None),
    DM_variable.PMAX:      (0.0, None),
    DM_variable.VMAX:      (0.0, None),
    DM_variable.TMAX:      (0.0, None),
    DM_variable.I_BW:      (100.0, 10000.0),
    DM_variable.KP_ASR:    (0.0, None),
    DM_variable.KI_ASR:    (0.0, None),
    DM_variable.KP_APR:    (0.0, None),
    DM_variable.KI_APR:    (0.0, None),
    DM_variable.GREF:      (0.0, 1.0),
    DM_variable.Deta:      (1.0, 30.0),
    DM_variable.V_BW:      (0.0, 500.0),
    DM_variable.IQ_c1:     (100.0, 10000.0),
    DM_variable.VL_c1:     (0.0, 10000.0),
    DM_variable.MST_ID:    (0, 0x7FF),
    DM_variable.ESC_ID:    (0, 0x7FF),
    DM_variable.TIMEOUT:   (0, None),
    DM_variable.CTRL_MODE: (1, 4),
}


def range_text(enum):
    if enum not in RANGE:
        return ""
    lo, hi = RANGE[enum]
    if lo is not None and hi is not None:
        return f"[{lo}, {hi}]"
    if lo is not None:
        return f"≥ {lo}"
    if hi is not None:
        return f"≤ {hi}"
    return ""


def out_of_range(enum, value):
    """返回越界提示文本，未越界返回 None。"""
    if enum not in RANGE:
        return None
    lo, hi = RANGE[enum]
    if (lo is not None and value < lo) or (hi is not None and value > hi):
        return f"超出建议范围 {range_text(enum)}"
    return None


# 名称 / RID 快速索引
BY_NAME = {p[0].name.lower(): p for p in PARAMS}
BY_RID = {int(p[0]): p for p in PARAMS}
CALIB_GROUP = "工厂校准"

CTRL_MODE_NAMES = {m.name.lower(): m for m in Control_Type}
CTRL_MODE_NAMES.update({str(int(m)): m for m in Control_Type})

# --------------------------------------------------------------------------
# 实时反馈量（不是寄存器，来自 CAN 反馈帧）。用于在表格中以“实时”类型展示。
# (行id, 名称, 单位, mon_vars键, 说明)
# --------------------------------------------------------------------------
RT_FIELDS = [
    ("RT_q",     "位置 q",   "rad",   "q",
     "电机当前位置（实时）。来自 CAN 反馈帧，经 refresh_motor_status 读取，不使能电机。"),
    ("RT_dq",    "速度 dq",  "rad/s", "dq",
     "电机当前角速度（实时）。"),
    ("RT_tau",   "力矩 τ",   "Nm",    "tau",
     "电机当前输出力矩/电流估计（实时）。"),
    ("RT_tmos",  "MOS温度",  "℃",     "tmos",
     "驱动 MOS 温度（实时）。"),
    ("RT_troto", "线圈温度", "℃",     "troto",
     "电机线圈温度（实时）。"),
]
RT_GROUP = "实时反馈"
RT_LEVEL = "实时"
RT_INFO = {iid: (label, unit, desc) for iid, label, unit, _k, desc in RT_FIELDS}
RT_KEY = {iid: k for iid, _l, _u, k, _d in RT_FIELDS}


# --------------------------------------------------------------------------
# 连接
# --------------------------------------------------------------------------
def connect(port, baud, esc_id, mst_id, motor_type):
    ser = serial.Serial(port, baud, timeout=0.5)
    mc = MotorControl(ser)
    motor = Motor(motor_type, esc_id, mst_id)
    mc.addMotor(motor)
    return ser, mc, motor


def resolve(token):
    """把用户输入的名称或 RID 解析为寄存器表条目。"""
    t = str(token).strip().lower()
    if t in BY_NAME:
        return BY_NAME[t]
    if t.isdigit() and int(t) in BY_RID:
        return BY_RID[int(t)]
    return None


def fmt_value(val):
    if val is None:
        return "—(无响应)"
    if isinstance(val, float):
        return f"{val:.6g}"
    return str(val)


# --------------------------------------------------------------------------
# 读
# --------------------------------------------------------------------------
def read_one(mc, motor, entry):
    enum, _, _, _, _ = entry
    return mc.read_motor_param(motor, enum)


def cmd_dump(mc, motor, include_calib=False):
    print("\n========== 电机参数读取 (read-only) ==========")
    print(f"  ESC_ID(SlaveID)=0x{motor.SlaveID:X}  MasterID=0x{motor.MasterID:X}\n")
    groups = []
    for entry in PARAMS:
        g = entry[1]
        if g not in groups:
            groups.append(g)
    ok = total = 0
    for g in groups:
        if g == CALIB_GROUP and not include_calib:
            continue
        print(f"【{g}】")
        for entry in PARAMS:
            enum, grp, unit, level, desc = entry
            if grp != g:
                continue
            total += 1
            val = read_one(mc, motor, entry)
            if val is not None:
                ok += 1
            unit_s = f" {unit}" if unit else ""
            print(f"  {enum.name:10s} RID{int(enum):>2d} [{level:5s}] = "
                  f"{fmt_value(val)}{unit_s:8s}  {desc}")
        print()
    if not include_calib:
        print("（工厂校准寄存器已隐藏，使用 dump --all 查看）")
    print(f"---- 成功读取 {ok}/{total} 个参数 ----\n")


def cmd_get(mc, motor, token):
    entry = resolve(token)
    if entry is None:
        print(f"未知参数: {token}（可用 dump 查看全部）")
        return False
    enum, grp, unit, level, desc = entry
    val = read_one(mc, motor, entry)
    unit_s = f" {unit}" if unit else ""
    print(f"{enum.name} (RID {int(enum)}, {grp}, {level}) = {fmt_value(val)}{unit_s}")
    print(f"  说明: {desc}")
    return val is not None


# --------------------------------------------------------------------------
# 写
# --------------------------------------------------------------------------
def _parse_value_for(entry, raw):
    """按寄存器类型把字符串转成 int / float；CTRL_MODE 支持模式名。"""
    enum = entry[0]
    if enum == DM_variable.CTRL_MODE:
        key = str(raw).strip().lower()
        if key in CTRL_MODE_NAMES:
            return int(CTRL_MODE_NAMES[key])
        raise ValueError("CTRL_MODE 取值: MIT/POS_VEL/VEL/Torque_Pos 或 1-4")
    if is_in_ranges(int(enum)):
        return int(float(raw))  # 容忍 "8.0" 这种输入
    return float(raw)


def cmd_set(mc, motor, token, raw_value, force=False, do_save=False, assume_yes=False,
            out=print):
    entry = resolve(token)
    if entry is None:
        out(f"未知参数: {token}")
        return False
    enum, grp, unit, level, desc = entry

    if level == RO:
        out(f"[拒绝] {enum.name} 属 {grp}（只读/标定），不允许写入，以免损坏电机标定。")
        return False

    try:
        value = _parse_value_for(entry, raw_value)
    except ValueError as e:
        out(f"[错误] 数值解析失败: {e}")
        return False

    # 安全边界提示
    if enum.name in SAFE_HINT:
        title, hint = SAFE_HINT[enum.name]
        out(f"[提示] {title}: {hint}")

    # 取值范围校验（超范围电机会拒绝写入）
    oor = out_of_range(enum, value)
    if oor:
        out(f"[范围警告] {enum.name} = {value} {oor}（合法范围 {range_text(enum)}）。"
            f"电机很可能拒绝该写入。")
        if not force and not assume_yes:
            out("  如确实要尝试，请追加 --force。")
            return False

    if level in SENSITIVE and not force and not assume_yes:
        out(f"[需要确认] {enum.name} 属敏感类别 [{level}]。"
            f"CLI 写入请追加 --force（已了解风险）。")
        return False

    cur = read_one(mc, motor, entry)
    out(f"写入 {enum.name} (RID {int(enum)}) : {fmt_value(cur)} -> {value}{(' ' + unit) if unit else ''}")

    # CTRL_MODE 走专用接口（带回读校验）
    if enum == DM_variable.CTRL_MODE:
        ok = mc.switchControlMode(motor, Control_Type(value))
    else:
        ok = mc.change_motor_param(motor, enum, value)

    if ok:
        new = read_one(mc, motor, entry)
        out(f"  [成功] 当前值 = {fmt_value(new)}（注意：未保存，断电将丢失）")
        if do_save:
            cmd_save(mc, motor, out=out)
        elif level == COMM and enum in (DM_variable.ESC_ID, DM_variable.MST_ID, DM_variable.can_br):
            out("  [警告] 通讯参数已改，保存并重新上电后需用新 ID/波特率重新连接！")
        return True
    out("  [失败] 写入未确认。可能原因：① 超出取值范围被拒；② 该寄存器只读；"
        "③ 此固件不支持该 RID（无回应）。")
    return False


def cmd_save(mc, motor, assume_yes=True, out=print):
    out("保存全部参数到 Flash（会先 disable 电机）...")
    mc.save_motor_param(motor)
    time.sleep(0.2)
    out("  [完成] 已发送保存指令。建议重新上电验证。")
    return True


def cmd_mode(mc, motor, mode_token, do_save=False, out=print):
    key = str(mode_token).strip().lower()
    if key not in CTRL_MODE_NAMES:
        out("模式取值: MIT / POS_VEL / VEL / Torque_Pos 或 1-4")
        return False
    mode = CTRL_MODE_NAMES[key]
    out(f"切换控制模式 -> {mode.name} ({int(mode)}) ...")
    ok = mc.switchControlMode(motor, mode)
    out("  [成功]" if ok else "  [失败]")
    if ok and do_save:
        cmd_save(mc, motor, out=out)
    return ok


def cmd_export(mc, motor, path, include_calib=True, out=print):
    rows = []
    for entry in PARAMS:
        enum, grp, unit, level, desc = entry
        if grp == CALIB_GROUP and not include_calib:
            continue
        val = read_one(mc, motor, entry)
        rows.append([enum.name, "静态", int(enum), grp, level, unit,
                     "" if val is None else val, desc])
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["name", "type", "RID", "group", "level", "unit", "value", "desc"])
        w.writerows(rows)
    out(f"已导出 {len(rows)} 个参数到 {path}")
    return True


# --------------------------------------------------------------------------
# 交互菜单
# --------------------------------------------------------------------------
def interactive(mc, motor):
    print("\n=== DM4310 参数配置 · 交互模式 ===")
    print("命令: dump | dumpall | get <名/RID> | set <名/RID> <值> | mode <模式>")
    print("      save | export <文件> | specs | list | help | quit\n")
    while True:
        try:
            line = input("param> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            break
        elif cmd in ("help", "h", "?"):
            print("dump | dumpall | get <名/RID> | set <名/RID> <值> | mode <模式> |"
                  " save | export <文件> | specs | list | quit")
        elif cmd == "list":
            for entry in PARAMS:
                enum, grp, unit, level, desc = entry
                print(f"  {enum.name:10s} RID{int(enum):>2d} [{level:5s}] {grp:8s} {desc}")
        elif cmd == "specs":
            print("出厂铭牌规格:")
            for k, v in SPECS.items():
                print(f"  {k}: {v}")
        elif cmd == "dump":
            cmd_dump(mc, motor, include_calib=False)
        elif cmd == "dumpall":
            cmd_dump(mc, motor, include_calib=True)
        elif cmd == "get" and len(parts) >= 2:
            cmd_get(mc, motor, parts[1])
        elif cmd == "mode" and len(parts) >= 2:
            cmd_mode(mc, motor, parts[1])
        elif cmd == "save":
            if input("确认保存到 Flash? (y/N) ").strip().lower() == "y":
                cmd_save(mc, motor)
        elif cmd == "export" and len(parts) >= 2:
            cmd_export(mc, motor, parts[1])
        elif cmd == "set" and len(parts) >= 3:
            entry = resolve(parts[1])
            confirm = True
            if entry is not None and entry[3] in SENSITIVE:
                confirm = input(f"[{entry[3]}] 敏感参数 {entry[0].name}，确认写入? (y/N) "
                                ).strip().lower() == "y"
            if confirm:
                cmd_set(mc, motor, parts[1], parts[2], assume_yes=True)
            else:
                print("已取消。")
        else:
            print("无法识别，输入 help 查看用法。")
    print("退出交互模式。")


# --------------------------------------------------------------------------
# 图形界面（Tkinter）：参数表 + 读/写/保存/切模式/导出，串口操作在后台线程
# --------------------------------------------------------------------------
def launch_gui(args):
    import queue
    import threading
    import tkinter as tk
    from tkinter import ttk, messagebox, filedialog

    LEVEL_COLOR = {
        RO: "#9aa0a6",      # 灰：只读
        RW: "#1b5e20",      # 绿：常规可写
        LIMIT: "#e65100",   # 橙：运动上限
        PROT: "#b71c1c",    # 红：保护阈值
        COMM: "#0d47a1",    # 蓝：通讯/模式
    }

    class _Tooltip:
        """悬浮提示窗口，用于显示参数详细说明。"""

        def __init__(self, master):
            self.master = master
            self.win = None

        def show(self, text, x, y):
            self.hide()
            self.win = tk.Toplevel(self.master)
            self.win.wm_overrideredirect(True)
            self.win.wm_geometry(f"+{x}+{y}")
            self.win.attributes("-topmost", True)
            tk.Label(self.win, text=text, justify="left", background="#fffbe6",
                     foreground="#202020", relief="solid", borderwidth=1,
                     wraplength=440, font=("Microsoft YaHei", 9),
                     padx=8, pady=6).pack()

        def hide(self):
            if self.win is not None:
                self.win.destroy()
                self.win = None

    class ParamGui:
        def __init__(self, root):
            self.root = root
            self.conn = None            # (ser, mc, motor)
            self.task_q = queue.Queue()
            self.result_q = queue.Queue()
            self.busy = 0
            self.show_calib = tk.BooleanVar(value=False)
            self._tip = _Tooltip(root)
            self._tip_key = None
            self.monitor_on = tk.BooleanVar(value=False)
            self._mon_pending = False
            self.mon_vars = {k: tk.StringVar(value="—")
                             for k in ("q", "dq", "tau", "tmos", "troto")}

            root.title("DM4310 电机参数配置工具")
            root.geometry("1080x800")
            root.minsize(900, 640)

            self._build_conn(root)
            self._build_monitor(root)
            self._build_table(root)
            self._build_actions(root)
            self._build_log(root)

            self._worker = threading.Thread(target=self._worker_loop, daemon=True)
            self._worker.start()
            self.root.after(80, self._poll)
            self.root.protocol("WM_DELETE_WINDOW", self._on_close)
            self._set_connected(False)
            self.log("就绪。请先填写串口参数并点击「连接」。")

        # ---------- 布局 ----------
        def _build_conn(self, root):
            f = ttk.LabelFrame(root, text="连接")
            f.pack(fill="x", padx=8, pady=(8, 4))
            ttk.Label(f, text="串口").grid(row=0, column=0, padx=4, pady=6)
            self.e_port = ttk.Entry(f, width=8)
            self.e_port.insert(0, args.port)
            self.e_port.grid(row=0, column=1)
            ttk.Label(f, text="波特率").grid(row=0, column=2, padx=4)
            self.e_baud = ttk.Entry(f, width=9)
            self.e_baud.insert(0, str(args.baud))
            self.e_baud.grid(row=0, column=3)
            ttk.Label(f, text="ESC_ID").grid(row=0, column=4, padx=4)
            self.e_esc = ttk.Entry(f, width=7)
            self.e_esc.insert(0, hex(args.esc_id))
            self.e_esc.grid(row=0, column=5)
            ttk.Label(f, text="MST_ID").grid(row=0, column=6, padx=4)
            self.e_mst = ttk.Entry(f, width=7)
            self.e_mst.insert(0, hex(args.mst_id))
            self.e_mst.grid(row=0, column=7)
            self.btn_conn = ttk.Button(f, text="连接", command=self._toggle_conn)
            self.btn_conn.grid(row=0, column=8, padx=8)
            self.lbl_status = ttk.Label(f, text="未连接", foreground="#b71c1c")
            self.lbl_status.grid(row=0, column=9, padx=4)

        def _build_monitor(self, root):
            f = ttk.LabelFrame(root, text="实时状态 (只读 · 来自 CAN 反馈帧, 不使能电机)")
            f.pack(fill="x", padx=8, pady=(0, 4))
            self.chk_mon = ttk.Checkbutton(f, text="实时刷新", variable=self.monitor_on,
                                           command=self._toggle_monitor)
            self.chk_mon.grid(row=0, column=0, padx=6, pady=6)
            cells = [("位置 q", "q", "rad"), ("速度 dq", "dq", "rad/s"),
                     ("力矩 τ", "tau", "Nm"), ("MOS温度", "tmos", "℃"),
                     ("线圈温度", "troto", "℃")]
            col = 1
            for label, key, unit in cells:
                ttk.Label(f, text=f"{label}:").grid(row=0, column=col, padx=(10, 2))
                ttk.Label(f, textvariable=self.mon_vars[key], width=9,
                          foreground="#0d47a1", font=("Consolas", 11, "bold")
                          ).grid(row=0, column=col + 1)
                ttk.Label(f, text=unit, foreground="#888").grid(row=0, column=col + 2)
                col += 3

        def _build_table(self, root):
            f = ttk.Frame(root)
            f.pack(fill="both", expand=True, padx=8, pady=4)
            cols = ("name", "type", "rid", "group", "level", "unit", "value", "desc")
            heads = {"name": "名称", "type": "数据类型", "rid": "RID", "group": "分组",
                     "level": "级别", "unit": "单位", "value": "当前值",
                     "desc": "说明 (悬停查看详情)"}
            widths = {"name": 105, "type": "70", "rid": 45, "group": 78, "level": 55,
                      "unit": 52, "value": 105, "desc": 300}
            self.tree = ttk.Treeview(f, columns=cols, show="headings", height=18)
            for c in cols:
                self.tree.heading(c, text=heads[c])
                self.tree.column(c, width=int(widths[c]),
                                 anchor=("center" if c in ("type", "rid", "level", "unit")
                                         else "w"))
            vsb = ttk.Scrollbar(f, orient="vertical", command=self.tree.yview)
            self.tree.configure(yscrollcommand=vsb.set)
            self.tree.pack(side="left", fill="both", expand=True)
            vsb.pack(side="right", fill="y")
            for lvl, color in LEVEL_COLOR.items():
                self.tree.tag_configure(lvl, foreground=color)
            self.tree.tag_configure(RT_LEVEL, foreground="#6a1b9a", background="#f3e9fb")
            self.tree.bind("<<TreeviewSelect>>", self._on_select)
            self.tree.bind("<Double-1>", lambda e: self._read_selected())
            self.tree.bind("<Motion>", self._on_hover)
            self.tree.bind("<Leave>", lambda e: self._hide_tip())
            self._populate_rows()

        def _build_actions(self, root):
            f = ttk.LabelFrame(root, text="操作")
            f.pack(fill="x", padx=8, pady=4)
            self.btn_read_all = ttk.Button(f, text="读取全部", command=self._read_all)
            self.btn_read_all.grid(row=0, column=0, padx=4, pady=6)
            self.btn_read_sel = ttk.Button(f, text="读取选中", command=self._read_selected)
            self.btn_read_sel.grid(row=0, column=1, padx=4)
            ttk.Checkbutton(f, text="显示工厂校准", variable=self.show_calib,
                            command=self._populate_rows).grid(row=0, column=2, padx=8)
            self.btn_save = ttk.Button(f, text="保存到 Flash", command=self._save)
            self.btn_save.grid(row=0, column=3, padx=4)
            self.btn_export = ttk.Button(f, text="导出 CSV", command=self._export)
            self.btn_export.grid(row=0, column=4, padx=4)
            ttk.Label(f, text="控制模式").grid(row=0, column=5, padx=(16, 2))
            self.cb_mode = ttk.Combobox(f, width=10, state="readonly",
                                        values=[m.name for m in Control_Type])
            self.cb_mode.set("MIT")
            self.cb_mode.grid(row=0, column=6)
            self.btn_mode = ttk.Button(f, text="应用模式", command=self._apply_mode)
            self.btn_mode.grid(row=0, column=7, padx=4)

            # 写入区
            w = ttk.Frame(f)
            w.grid(row=1, column=0, columnspan=8, sticky="w", pady=(4, 6))
            self.lbl_sel = ttk.Label(w, text="选中: —", width=46)
            self.lbl_sel.pack(side="left", padx=4)
            ttk.Label(w, text="新值").pack(side="left")
            self.e_val = ttk.Entry(w, width=16)
            self.e_val.pack(side="left", padx=4)
            self.e_val.bind("<Return>", lambda e: self._write_selected())
            self.btn_write = ttk.Button(w, text="写入选中参数", command=self._write_selected)
            self.btn_write.pack(side="left", padx=4)

        def _build_log(self, root):
            f = ttk.LabelFrame(root, text="日志")
            f.pack(fill="both", padx=8, pady=(4, 8))
            self.txt = tk.Text(f, height=8, wrap="word")
            sb = ttk.Scrollbar(f, orient="vertical", command=self.txt.yview)
            self.txt.configure(yscrollcommand=sb.set, state="disabled")
            self.txt.pack(side="left", fill="both", expand=True)
            sb.pack(side="right", fill="y")

        # ---------- 行 ----------
        def _populate_rows(self):
            for iid in self.tree.get_children():
                self.tree.delete(iid)
            # 实时反馈行（置顶，类型=实时）
            for iid, label, unit, _k, desc in RT_FIELDS:
                self.tree.insert("", "end", iid=iid,
                                 values=(label, RT_LEVEL, "", RT_GROUP, "RT", unit, "—", desc),
                                 tags=(RT_LEVEL,))
            # 静态寄存器行
            for entry in PARAMS:
                enum, grp, unit, level, desc = entry
                if grp == CALIB_GROUP and not self.show_calib.get():
                    continue
                self.tree.insert("", "end", iid=enum.name,
                                 values=(enum.name, "静态", int(enum), grp, level, unit, "", desc),
                                 tags=(level,))

        def _selected_iid(self):
            sel = self.tree.selection()
            return sel[0] if sel else None

        def _selected_entry(self):
            iid = self._selected_iid()
            return BY_NAME.get(iid.lower()) if iid else None

        def _on_select(self, _evt=None):
            iid = self._selected_iid()
            if iid in RT_INFO:
                label = RT_INFO[iid][0]
                self.lbl_sel.config(text=f"选中: {label} [实时反馈 · 只读]")
                self.e_val.delete(0, "end")
                return
            entry = self._selected_entry()
            if not entry:
                self.lbl_sel.config(text="选中: —")
                return
            enum, grp, unit, level, desc = entry
            cur = self.tree.set(enum.name, "value")
            self.lbl_sel.config(text=f"选中: {enum.name} [{level}] 当前={cur or '—'}")
            self.e_val.delete(0, "end")
            if level != RO and cur:
                self.e_val.insert(0, cur)

        def _hide_tip(self):
            self._tip_key = None
            self._tip.hide()

        def _on_hover(self, e):
            row = self.tree.identify_row(e.y)
            col = self.tree.identify_column(e.x)
            # 仅在“说明”列(第 8 列 = #8)悬停时弹出
            if not row or col != "#8":
                if self._tip_key is not None:
                    self._hide_tip()
                return
            key = (row, col)
            if key == self._tip_key:
                return
            self._tip_key = key
            if row in RT_INFO:
                label, unit, desc = RT_INFO[row]
                head = f"{label}  ·  {RT_GROUP} · 实时(只读)  ·  单位 {unit}"
                self._tip.show(f"{head}\n{'─' * 24}\n{desc}\n"
                               "数据来源：CAN 反馈帧，非寄存器。勾选『实时刷新』后周期更新。",
                               e.x_root + 18, e.y_root + 14)
                return
            entry = BY_NAME.get(row.lower())
            if not entry:
                self._hide_tip()
                return
            enum, grp, unit, level, desc = entry
            unit_s = f" · 单位 {unit}" if unit else ""
            head = f"{enum.name}  (RID {int(enum)}) · {grp} · 级别 {level}{unit_s}"
            detail = DETAILS.get(enum, desc)
            rng = range_text(enum)
            rng_s = f"\n合法范围: {rng}" if rng else ""
            rw_s = "\n【只读, 不可写】" if level == RO else ""
            self._tip.show(f"{head}\n{'─' * 24}\n{detail}{rng_s}{rw_s}",
                           e.x_root + 18, e.y_root + 14)

        # ---------- 线程/队列 ----------
        def log(self, msg):
            self._post(lambda: self._append(msg))

        def _append(self, msg):
            self.txt.configure(state="normal")
            self.txt.insert("end", str(msg) + "\n")
            self.txt.see("end")
            self.txt.configure(state="disabled")

        def _post(self, fn):
            self.result_q.put(fn)

        def _submit(self, name, fn, track_busy=True):
            def job():
                if track_busy:
                    self._post(lambda: self._set_busy(+1, name))
                try:
                    fn()
                except Exception as e:  # noqa: BLE001
                    self.log(f"[异常] {name}: {e}")
                finally:
                    if track_busy:
                        self._post(lambda: self._set_busy(-1, name))
            self.task_q.put(job)

        def _worker_loop(self):
            while True:
                job = self.task_q.get()
                if job is None:
                    break
                job()

        def _poll(self):
            try:
                while True:
                    fn = self.result_q.get_nowait()
                    fn()
            except queue.Empty:
                pass
            self.root.after(80, self._poll)

        def _set_busy(self, delta, name):
            self.busy = max(0, self.busy + delta)
            if self.busy and self.conn:
                self.lbl_status.config(text=f"忙: {name}", foreground="#e65100")
            elif self.conn:
                self.lbl_status.config(text="已连接 · 就绪", foreground="#1b5e20")

        def _set_connected(self, on):
            state = "normal" if on else "disabled"
            for b in (self.btn_read_all, self.btn_read_sel, self.btn_save,
                      self.btn_export, self.btn_mode, self.btn_write, self.chk_mon):
                b.config(state=state)
            self.btn_conn.config(text="断开" if on else "连接")
            self.lbl_status.config(text="已连接 · 就绪" if on else "未连接",
                                   foreground="#1b5e20" if on else "#b71c1c")

        # ---------- 业务 ----------
        def _toggle_conn(self):
            if self.conn:
                self.monitor_on.set(False)
                for v in self.mon_vars.values():
                    v.set("—")
                self._clear_rt_rows()
                ser, _, _ = self.conn
                self.conn = None
                self._set_connected(False)
                try:
                    ser.close()
                except Exception:
                    pass
                self.log("已断开。")
                return
            try:
                port = self.e_port.get().strip()
                baud = int(self.e_baud.get().strip())
                esc = int(self.e_esc.get().strip(), 0)
                mst = int(self.e_mst.get().strip(), 0)
            except ValueError as e:
                messagebox.showerror("参数错误", f"连接参数无效: {e}")
                return

            def do():
                try:
                    ser, mc, motor = connect(port, baud, esc, mst, DM_Motor_Type.DM4310)
                except Exception as e:  # noqa: BLE001
                    self.log(f"[串口错误] 无法打开 {port}: {e}")
                    return
                self.conn = (ser, mc, motor)
                self._mon_pending = False
                self._post(lambda: self._set_connected(True))
                self.log(f"已连接 {port}@{baud}  ESC=0x{esc:X} MST=0x{mst:X}")
                self._read_all_job()
            self._submit("连接", do)

        def _read_all(self):
            if not self._require_conn():
                return
            self._submit("读取全部", self._read_all_job)

        def _read_all_job(self):
            if not self.conn:
                return
            _, mc, motor = self.conn
            n = ok = 0
            for entry in PARAMS:
                enum, grp = entry[0], entry[1]
                if grp == CALIB_GROUP and not self.show_calib.get():
                    continue
                n += 1
                val = read_one(mc, motor, entry)
                if val is not None:
                    ok += 1
                self._post(lambda e=enum, v=val: self.tree.set(e.name, "value", fmt_value(v)))
            self.log(f"读取完成: {ok}/{n}")

        def _read_selected(self):
            if not self._require_conn():
                return
            iid = self._selected_iid()
            if iid in RT_INFO:
                self._submit("读实时", self._monitor_once)
                return
            entry = self._selected_entry()
            if not entry:
                self.log("请先在表中选择一行。")
                return
            enum = entry[0]

            def do():
                _, mc, motor = self.conn
                val = read_one(mc, motor, entry)
                self._post(lambda: self.tree.set(enum.name, "value", fmt_value(val)))
                self.log(f"{enum.name} = {fmt_value(val)}")
                self._post(self._on_select)
            self._submit(f"读 {enum.name}", do)

        def _write_selected(self):
            if not self._require_conn():
                return
            iid = self._selected_iid()
            if iid in RT_INFO:
                messagebox.showinfo("实时数据", "该行是实时反馈量（来自 CAN 反馈帧），只读，不可写入。")
                return
            entry = self._selected_entry()
            if not entry:
                self.log("请先在表中选择一行。")
                return
            enum, grp, unit, level, desc = entry
            if level == RO:
                messagebox.showwarning("只读", f"{enum.name} 属 {grp}（只读/工厂标定），不允许写入。")
                return
            raw = self.e_val.get().strip()
            if raw == "":
                messagebox.showinfo("提示", "请输入新值。")
                return
            # 范围预检（避免超范围写入被电机静默拒绝）
            try:
                parsed = _parse_value_for(entry, raw)
                oor = out_of_range(enum, parsed)
            except ValueError:
                oor = None
            if oor:
                if not messagebox.askyesno(
                        "超出取值范围",
                        f"{enum.name} = {raw}\n{oor}（合法范围 {range_text(enum)}）。\n\n"
                        f"电机很可能拒绝该值。仍要尝试写入？"):
                    return
            if level in SENSITIVE:
                if not messagebox.askyesno(
                        "敏感参数确认",
                        f"{enum.name} 属敏感类别 [{level}]\n{desc}\n\n"
                        f"确认写入 {raw}? 改动可能影响安全/通讯。"):
                    return

            def do():
                _, mc, motor = self.conn
                ok = cmd_set(mc, motor, enum.name, raw, assume_yes=True, out=self.log)
                if ok:
                    new = read_one(mc, motor, entry)
                    self._post(lambda: self.tree.set(enum.name, "value", fmt_value(new)))
            self._submit(f"写 {enum.name}", do)

        def _save(self):
            if not self._require_conn():
                return
            if not messagebox.askyesno("保存", "保存全部参数到 Flash？\n会先 disable 电机，建议保存后重新上电。"):
                return
            self._submit("保存", lambda: cmd_save(self.conn[1], self.conn[2], out=self.log))

        def _apply_mode(self):
            if not self._require_conn():
                return
            mode = self.cb_mode.get()
            self._submit(f"切模式 {mode}",
                         lambda: (cmd_mode(self.conn[1], self.conn[2], mode, out=self.log),
                                  self._post(lambda: self.tree.set(
                                      "CTRL_MODE", "value",
                                      str(int(CTRL_MODE_NAMES[mode.lower()]))))))

        def _export(self):
            if not self._require_conn():
                return
            path = filedialog.asksaveasfilename(
                defaultextension=".csv", filetypes=[("CSV", "*.csv")],
                initialfile="dm4310_params.csv")
            if not path:
                return
            self._submit("导出", lambda: cmd_export(
                self.conn[1], self.conn[2], path,
                include_calib=self.show_calib.get(), out=self.log))

        def _toggle_monitor(self):
            if self.monitor_on.get():
                if not self.conn:
                    self.monitor_on.set(False)
                    messagebox.showinfo("未连接", "请先连接电机。")
                    return
                self.log("开始实时状态刷新（只读，不使能电机）。")
                self._monitor_tick()
            else:
                self.log("停止实时状态刷新。")
                for v in self.mon_vars.values():
                    v.set("—")
                self._clear_rt_rows()

        def _monitor_tick(self):
            if not self.monitor_on.get():
                return
            if self.conn and not self._mon_pending and self.busy == 0:
                self._mon_pending = True

                def do():
                    try:
                        _, mc, motor = self.conn
                        mc.refresh_motor_status(motor)
                        q, dq, tau = motor.getPosition(), motor.getVelocity(), motor.getTorque()
                        tmos, troto = int(motor.T_Mos), int(motor.T_Roto)
                        self._post(lambda: self._apply_monitor(q, dq, tau, tmos, troto))
                    finally:
                        self._post(lambda: setattr(self, "_mon_pending", False))
                self._submit("实时", do, track_busy=False)
            self.root.after(250, self._monitor_tick)

        def _monitor_once(self):
            """单次读取实时反馈（用于选中实时行后点『读取选中』）。"""
            if not self.conn:
                return
            _, mc, motor = self.conn
            mc.refresh_motor_status(motor)
            q, dq, tau = motor.getPosition(), motor.getVelocity(), motor.getTorque()
            tmos, troto = int(motor.T_Mos), int(motor.T_Roto)
            self._post(lambda: self._apply_monitor(q, dq, tau, tmos, troto))

        def _apply_monitor(self, q, dq, tau, tmos, troto):
            vals = {"q": f"{q:+.4f}", "dq": f"{dq:+.3f}", "tau": f"{tau:+.3f}",
                    "tmos": str(tmos), "troto": str(troto)}
            for key, text in vals.items():
                self.mon_vars[key].set(text)
            # 同步刷新表格中的“实时”行
            for iid, k in RT_KEY.items():
                if self.tree.exists(iid):
                    self.tree.set(iid, "value", vals[k])

        def _clear_rt_rows(self):
            for iid in RT_KEY:
                if self.tree.exists(iid):
                    self.tree.set(iid, "value", "—")

        def _require_conn(self):
            if not self.conn:
                messagebox.showinfo("未连接", "请先连接电机。")
                return False
            return True

        def _on_close(self):
            self.monitor_on.set(False)
            self._tip.hide()
            if self.conn:
                try:
                    self.conn[0].close()
                except Exception:
                    pass
            self.task_q.put(None)
            self.root.destroy()

    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass
    ParamGui(root)
    root.mainloop()


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(
        description="DM-J4310 电机参数配置工具（读取/设置/保存 寄存器，区别于 Gripper_Studio 的运行控制）")
    p.add_argument("--port", default="COM6", help="串口号 (默认 COM6)")
    p.add_argument("--baud", type=int, default=921600, help="波特率 (默认 921600)")
    p.add_argument("--esc-id", type=lambda x: int(x, 0), default=0x01, help="电机 CAN ID (默认 0x01)")
    p.add_argument("--mst-id", type=lambda x: int(x, 0), default=0x101, help="主机反馈 ID (默认 0x101)")
    p.add_argument("--force", action="store_true", help="允许写入敏感参数(LIMIT/PROT/COMM)")

    sub = p.add_subparsers(dest="cmd")
    d = sub.add_parser("dump", help="读取全部参数")
    d.add_argument("--all", action="store_true", help="包含工厂校准寄存器")

    g = sub.add_parser("get", help="读取单个参数")
    g.add_argument("name")

    s = sub.add_parser("set", help="设置单个参数")
    s.add_argument("name")
    s.add_argument("value")
    s.add_argument("--save", action="store_true", help="写入后保存到 Flash")

    m = sub.add_parser("mode", help="切换控制模式")
    m.add_argument("value", help="MIT/POS_VEL/VEL/Torque_Pos 或 1-4")
    m.add_argument("--save", action="store_true", help="切换后保存")

    sub.add_parser("save", help="保存全部参数到 Flash")

    e = sub.add_parser("export", help="读取并导出为 CSV")
    e.add_argument("path")

    sub.add_parser("specs", help="打印出厂铭牌规格（无需连接）")
    sub.add_parser("repl", help="启动文本交互菜单")
    return p


def main():
    args = build_parser().parse_args()

    # specs 无需连接硬件
    if args.cmd == "specs":
        print("出厂铭牌规格 (DM-J4310-2EC):")
        for k, v in SPECS.items():
            print(f"  {k}: {v}")
        return

    # 无子命令 -> 图形界面（连接在窗口内进行）
    if args.cmd is None:
        launch_gui(args)
        return

    try:
        ser, mc, motor = connect(args.port, args.baud, args.esc_id, args.mst_id,
                                 DM_Motor_Type.DM4310)
    except serial.SerialException as e:
        print(f"[串口错误] 无法打开 {args.port}: {e}")
        sys.exit(1)

    try:
        if args.cmd == "repl":
            interactive(mc, motor)
        elif args.cmd == "dump":
            cmd_dump(mc, motor, include_calib=args.all)
        elif args.cmd == "get":
            cmd_get(mc, motor, args.name)
        elif args.cmd == "set":
            cmd_set(mc, motor, args.name, args.value, force=args.force, do_save=args.save)
        elif args.cmd == "mode":
            cmd_mode(mc, motor, args.value, do_save=args.save)
        elif args.cmd == "save":
            cmd_save(mc, motor)
        elif args.cmd == "export":
            cmd_export(mc, motor, args.path)
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
