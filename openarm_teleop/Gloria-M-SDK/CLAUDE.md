# CLAUDE.md

本文件为 Claude / AI 编码助手在本仓库工作时提供项目级指引。阅读后即可快速理解项目结构、运行环境、硬件协议与安全注意事项。

## 项目简介

`Gloria-M-SDK` 是基于**达妙科技 DM-J4310-2EC（DM4310）**一体化伺服电机开发的**夹爪（gripper）控制 SDK**。
上位机通过 **USB 转串口 / USB 转 CAN** 与电机驱动器通信，封装了电机使能、参数读写、控制模式切换、状态反馈解析，并在此基础上实现夹爪的位置 / 速度 / 力矩（电流）控制与力矩保护策略。

- 平台：Windows 10
- 运行环境：Miniconda 虚拟环境 `xygripper`（Python 3.10）
- 通信口：当前硬件枚举为 `COM6`，串口波特率 `921600`
- 电机 CAN：1 Mbps（由 USB-CAN 适配器桥接，对上位机表现为串口帧）

## 目录结构

```
Gloria-M-SDK/
├── CLAUDE.md                 # 本文件：AI 助手项目指引
├── README.md                 # 面向用户的中文使用文档
├── requirements.txt          # Python 依赖（serial / pyserial，注：实际还需 numpy）
├── doc/
│   └── dm4310-*.pdf          # DM-J4310-2EC 使用说明书（协议、参数、保护）
└── motor/
    ├── CAN.py                # 核心 SDK：Motor / MotorControl 类、协议编解码、枚举
    ├── XuanYa_Comm.py        # MotorComm 单位换算辅助类 + gloria_max 行程常量
    ├── XY_Motor_Dome.py      # 演示：正弦轨迹 MIT 位置控制
    ├── TorqueGuard_Demo.py   # 演示：带状态机的力矩保护（NORMAL/PROTECT/RECOVERY）
    ├── Torque_Current_Demo.py# 演示：电流/力矩控制（MIT kp=0 纯力矩 + 速度阻尼保护 + 梯形力矩曲线）
    ├── Gripper_Calib.py      # 行程标定：小力矩试探两端机械限位, 测行程范围与开/合方向
    ├── Gripper_Keyboard_Ctrl.py # 键盘实时控制：行程(位置)模式 + 夹持力(力矩)模式; 带 --selftest 无键盘自检
    ├── Gripper_Studio.py     # GUI 工作站：实时曲线 + 控制面板 + 保护调试 + CSV 记录 (Tkinter+matplotlib)
    ├── Motor_Param_Tool.py   # 电机参数配置工具：读/写/保存寄存器 + 切模式 + CSV 导出 (CLI+交互)
    ├── Collision_Impedance_Lab.py # 碰撞/阻抗/零位 测试台：阻抗(MIT)控制 + 碰撞等级检测 + 设零位 (GUI)
    ├── Gripper_Repeatability_Lab.py # 行程重复定位精度测试台：A⇄B 往返N次 + 到位统计(σ/极差) + 散点图 (GUI)
    └── conn_test.py          # 只读连通性自检：读取电机参数验证串口/CAN 通信
```

> 注意：`motor/` 内脚本使用 `from CAN import *`、`from XuanYa_Comm import *` 的同级相对导入，因此**必须在 `motor/` 目录下运行**（或将其加入 `sys.path`）。

## 运行环境与命令

由于 `conda run` 在中文 Windows 控制台下可能因 GBK 编码报 `UnicodeEncodeError`，推荐直接使用环境内的 Python 解释器全路径，并设置 UTF-8 输出：

```powershell
$env:PYTHONIOENCODING="utf-8"
& "C:\Users\KNOW\.conda\envs\xygripper\python.exe" <脚本>
```

常用操作：

```powershell
# 列出可用串口
& "C:\Users\KNOW\.conda\envs\xygripper\python.exe" -c "import serial.tools.list_ports as lp; [print(p.device,'-',p.description) for p in lp.comports()]"

# 运行正弦轨迹演示（在 motor/ 目录下）
cd motor
& "C:\Users\KNOW\.conda\envs\xygripper\python.exe" XY_Motor_Dome.py
```

依赖安装：

```powershell
& "C:\Users\KNOW\.conda\envs\xygripper\python.exe" -m pip install -r requirements.txt
& "C:\Users\KNOW\.conda\envs\xygripper\python.exe" -m pip install numpy
```

## 核心架构

### `CAN.py`

- **`Motor`**：单个电机的数据对象，保存期望/反馈的位置 `state_q`、速度 `state_dq`、力矩 `state_tau`、温度 `T_Mos`/`T_Roto`、控制模式与读回的参数字典 `temp_param_dict`。
- **`MotorControl`**：通信与控制核心，持有 `serial` 对象。
  - 控制：`controlMIT(motor, kp, kd, q, dq, tau)`、`control_Pos_Vel`、`control_Vel`、`control_pos_force`（力位混合 / 电流给定）。
  - 使能：`enable` / `enable_old`（旧固件需带控制模式偏移）/ `disable` / `set_zero_position`。
  - 参数：`read_motor_param` / `change_motor_param` / `switchControlMode` / `save_motor_param` / `change_limit_param` / `refresh_motor_status`。
  - 收发：`__send_data` 组帧（30 字节串口帧，帧头 `0x55 0xAA`），`recv` + `__extract_packets`（16 字节帧，头 `0xAA` 尾 `0x55`）+ `__process_packet` 解析反馈。
- **协议编解码**：`float_to_uint` / `uint_to_float`（线性映射，位置 16 位、速度/力矩 12 位）、`float_to_uint8s`、`LIMIT_MIN_MAX`（已修复为返回截断值）。
- **`Limit_Param`**：各电机型号的 `[P_MAX, V_MAX, T_MAX]`。DM4310 为 `[3.14, 20, 12]`，控制/反馈的物理量量程由此决定，**必须与电机内实际设置一致**，否则力矩/位置数值错误。
- **枚举**：`DM_Motor_Type`、`DM_variable`（RID 参数表）、`Control_Type`（MIT=1, POS_VEL=2, VEL=3, Torque_Pos=4）。

### `XuanYa_Comm.py`

- **`gloria_max = -2.9`**：夹爪行程常量（rad，负值表示夹合方向）。轨迹通常以 `gloria_max/2` 为中点、`gloria_max/2` 为幅度。
- **`MotorComm`**：单位换算辅助——`getPosRad`/`getAngle`/`getAngle100`（行程映射到 0~100）、`getVelRad`/`getVel`（rad→°/s）、`getTorque`、`getTMos`/`getTRoto`。

## 控制模式与电流 / 力矩控制要点

依据 DM4310 说明书（`doc/`）：

- **MIT 模式**：电调把 CAN 数据算出的扭矩值作为**电流环的电流给定**，故力矩控制本质即电流控制。
  - `kp=0, kd=0, tau=T_ff` → **纯力矩 / 电流输出**（夹爪夹持力控制的关键）。
  - `kp=0, kd≠0, dq` → 匀速。
  - 位置控制时 **kd 不可为 0**，否则电机震荡甚至失控。
- **力位混合（`control_pos_force`，Torque_Pos）**：可给定 `i_des`（电流标幺值，0~10000，实际电流/最大电流 ×10000），实现限流抓取。
- **量纲与量程**：力矩范围由 `Limit_Param[DM4310][2] = 12`（±12 Nm 映射）控制；DM4310 实际**额定扭矩 3 Nm、峰值 7 Nm**，**额定电流 2.5 A、峰值 7.5 A**。调试夹持力时务必从小力矩起步。

## 硬件 / 安全注意事项

- 上电后等待 1~2 秒再使能；运行结束务必 `disable` 并 `serial.close()`。
- 力矩 / 电流控制会让夹爪持续输出力，调试时**从 0.2~0.5 Nm 小值起步**，确认方向与量程后再逐步增大，避免堵转过热（电机过温保护 ~100℃）。
- 反馈帧 `data[0]>>4` 的故障码：8 超压 / 9 欠压 / A 过流 / B MOS 过温 / C 线圈过温 / D 通讯丢失 / E 过载。
- 修改限位等关键参数后需 `save_motor_param` 并重启电机生效。
- `gloria_max` 为负，注意控制目标符号，避免向错误方向冲程。

## 已知问题 / 调试记录（实测 COM6）

实机连通自检读回（`conn_test.py`）：`CTRL_MODE=1(MIT)`, `Gr=10`, `PMAX=3.14`, `VMAX=10.0`, `TMAX=12.0`, `KT_Value=0.0`, `MST_ID=0x101`, `ESC_ID=1`。

- **VMAX 量纲不一致**：SDK `Limit_Param[DM4310]` 默认 `V_MAX=20`，但电机实际 `VMAX=10`，会导致速度收发数值相差 2×。
  - 解决：运行时读取电机 PMAX/VMAX/TMAX 并 `change_limit_param` 同步（见 `Torque_Current_Demo.sync_limits`）。
- **温度反馈恒为 0（已修复）**：`CAN.py __process_packet` 原先把 `T_Mos/T_Roto` 写到 `MotorControl` 实例上，导致 `MotorComm.getTMos/getTRoto`（读 `Motor` 对象）永远为 0。已改为写入 `self.motors_map[CANID]`（及 MasterID 分支）。修复后实测 `Tmos≈31℃ Troto≈29℃`。
- **KT 未标定**：`KT_Value=0`，因此无法用 `电流=力矩/KT` 反推相电流，调试以力矩反馈为准；如需电流读数需先在调试助手标定 KT。
- **力矩控制实测**：`Torque_Current_Demo.py` 目标 0.3Nm，反馈力矩与指令吻合良好；纯力矩(kp=0)空载会持续运动，已用 `kd_safety=0.8` 阻尼 + `vel_limit` 速度切断保护。

### 夹爪行程标定（`Gripper_Calib.py` 实测，空载）

- **OPEN 全开端 ≈ -0.31 rad**，**CLOSE 全合端 ≈ -2.73 rad**，行程 **2.42 rad / 138.6°**，中点 ≈ -1.52 rad。
- **方向：负方向=闭合，正方向=张开**（与 `gloria_max=-2.9` 闭合参考一致）。
- 起步摩擦约 **0.3 Nm**：探针力矩需 ≥0.5 Nm 才能顶到限位。
- `getAngle100 = pos/gloria_max*100`：全开≈12%，全合≈92%（数值≈“闭合程度%”）。

### 行程 / 夹持力控制（`Gripper_Keyboard_Ctrl.py`）

- 行程(位置)模式：MIT `kp=12,kd=1.0` + 目标限速平滑(`MAX_SLEW`)，端点留安全余量 `OPEN_END=-0.35 / CLOSE_END=-2.70`。`--selftest` 实测 OPEN/CLOSE/MID 三点到位平滑、残余力矩小。
- 夹持力模式：MIT `kp=0,kd=0.8`，夹持=闭合方向=负力矩，力档限 0~0.8Nm（小力测试）。
- 键位：`o/c/m` 开/合/中，`a/d` 微调，`p/f` 切模式，`g/r` 夹持/松开，`+/-` 调力，`q/ESC` 退出。需在**真实终端**运行（依赖 `msvcrt`）。

### GUI 工作站（`Gripper_Studio.py`）

专业可视化调试工具，运行：`python Gripper_Studio.py`（依赖 `matplotlib`，已加入 `requirements.txt`）。

- **架构**：串口控制环跑在后台 `MotorWorker` 线程（独占串口，200Hz），GUI 主线程仅通过加锁的共享参数字典 + 命令队列交互，遥测经 `deque` 缓冲；GUI 每 40ms 刷新曲线。**所有 Tk 调用只在主线程**，串口只在 worker 线程。
- **曲线区**：4 个子图——开度%、速度、力矩/电流、温度；含保护限值参考线、`tau指令` vs `tau实测`、保护触发区间红色阴影。
- **控制面板**：连接/使能/设零位/标定/**初始化自检**；三种模式（位置·行程 / 夹持力 / MIT 原始）；开/合/中一键；目标/**速度(rad/s)**/Kp/Kd/力矩滑条。
- **速度调节**：位置·行程模式下「速度(rad/s)」滑条控制目标限速平滑的速率（参数 `speed`, 范围 0.2~8 rad/s, 默认 2.4）；内部换算为每周期最大步长 `max_slew = speed * SEND_DT`。值越小动作越柔和、越大越快。
- **初始化自检（恢复一致性）**：`设零位` 会把当前位置设为新 0 点、整体偏移行程坐标, 导致硬编码的 OPEN/CLOSE 限位失效、行程控制错乱。点「初始化自检」即可恢复: 同步量程/电气参数 → 使能 → 重新标定(顶到两端机械限位, 在**当前坐标系**重锚定 OPEN/CLOSE/MID) → 复位模式/目标/保护 → 回中点。`设零位` 与「初始化自检」均带确认弹窗。限位变更通过 `limits_version` 通知 GUI 同步刷新滑条范围（`_sync_slider_ranges`）。实测: 误标零后自检重锚定, 行程跨度仍正确(~2.46rad), 滑条范围随之更新。
- **保护调试**：速度/力矩/温度三类保护可独立开关并设阈值。温度超限→直接失能(FAULT)；速度超限→切位置保持(PROTECT)；力矩超限→过载方向冻结目标 / 夹持力被钳制。状态栏与读数实时显示。
- **CSV 记录**：一键记录 `t,pct,q,dq,tau,tau_cmd,Tmos,Troto,prot` 用于离线分析。
- **中文字体**：`matplotlib.rcParams["font.sans-serif"]` 设为 `Microsoft YaHei`，否则坐标轴中文显示方框。
- **关闭**：`_on_close` 会停 worker、`join` 收尾（失能+关串口）、`after_cancel` 防止 `can't delete Tcl command`。
- **注意**：异常退出若残留进程占用 COM6 会报“拒绝访问”，用任务管理器结束残留 `python.exe` 即可。
- **布局**：启动即最大化（`root.state("zoomed")`）；左侧菜单栏放在固定宽度容器内的可滚动 `Canvas` 中（`create_window`+`Scrollbar`+鼠标滚轮 `_on_mousewheel`，滚轮仅在指针位于左栏时生效），内容超出窗口高度可上下滚动。
- **电气数据面板**：
  - 重要事实：**DM4310 的 CAN 反馈帧只有 位置/速度/力矩/2×温度，没有实时母线电压与相电流**。
  - 实时电流为**估算值** `I = τ / KT`；`KT` 可在面板输入（缺省 1.2 Nm/A，由额定 3Nm/2.5A 推算；若 `KT_Value` 已标定>0 则连接时自动采用）。曲线力矩图右侧附加“电流 A(估算)”次坐标轴。
  - “读取电气参数”按钮读取并显示电机静态阈值：欠压 `UV_Value` / 过压 `OV_Value` / 过流 `OC_Value` / `KT_Value`（这些是配置阈值，非实时量）。
- **刷新性能（重要）**：实测单次 `canvas.draw()` 约 120ms（远大于早期 40ms 刷新间隔）。早期用 `draw_idle()`（绘制在独立 idle 任务）+ 高频定时器会导致**事件循环饱和、界面假死**。已改为**同步 `canvas.draw()`**，并在绘制完成后才 `after(REFRESH_MS=70)` 排程下一次刷新，保证任意机器上都有固定空闲间隔；限值参考线改为创建一次后原地更新（不再每帧 remove/add）。

### 参数配置工具（`Motor_Param_Tool.py`）

**定位**：与 `Gripper_Studio.py`（运行期“控制+曲线+保护”GUI）职责互补——本工具专注**电机内部寄存器参数的离线配置**（读取/设置/保存到 Flash/切控制模式），不做运动控制与可视化，避免功能重复。

- **图形界面（默认）**：直接 `python Motor_Param_Tool.py` 启动 Tkinter 窗口（**无 `gui` 子命令**，端口可用全局 `--port` 预填或在窗口内填写）——顶部连接区（串口/波特率/ESC_ID/MST_ID/连接）、中部参数表（`Treeview` 列出全部寄存器，按级别着色：RO灰/RW绿/LIMIT橙/PROT红/COMM蓝）、操作区（读取全部/读取选中/保存/导出/切模式 + 写入选中）、底部日志。**所有串口读写在后台 worker 线程**（`task_q`/`result_q` + `root.after` 轮询），UI 不卡死；写敏感参数（LIMIT/PROT/COMM）弹确认框、写 RO 直接拦截。双击行=读取该行。`repl` 子命令保留纯文本交互菜单。
- **悬停说明**：鼠标停在「说明」列某行时弹出该参数的详细解释（单位/出厂值/作用/注意事项），内容来自 `DETAILS`（全 45 个 RID 均有）。
- **静态参数 vs 实时数据（重要概念）**：45 个 `DM_variable` **全是静态寄存器**（配置/标定/信息），运行中不会自变，只有写入才变。**实时变化的位置/速度/力矩/温度不在寄存器里**，来自 CAN 反馈帧。
- **读写属性 & 取值范围（依官方寄存器表校正）**：`Damp`(11)/`Inertia`(12) 及 13-20(版本/Rs/Ls/Flux/NPP/Gr) **均为只读 RO**——“阻尼/惯量改不了”是正常现象，不是 bug。控制环增益 `I_BW`/`KP_ASR`/`KI_ASR`/`KP_APR`/`KI_APR`/`V_BW`/`IQ_c1`/`VL_c1` 及 `GREF`(齿轮力矩效率)/`Deta`(速度环阻尼系数) 为 **RW 可写**。`RANGE` 表内置各寄存器合法范围，**超范围写入会被电机静默拒绝**——典型坑：`OC_Value` 是 **0~1.0 标幺比例（触发电流/最大电流），不是安培**；`DEC` 取负值。GUI 写入前做范围预检弹窗，CLI 超范围需 `--force`；写失败时提示三种可能（超范围/只读/固件不支持）。
- **「数据类型」列**：参数表新增「数据类型」列区分 `静态`（45 个寄存器）与 `实时`（CAN 反馈量）。表格顶部置入 5 行 `实时` 反馈（位置 q/速度 dq/力矩 τ/MOS温度/线圈温度，紫色高亮），勾选「实时刷新」后随监视面板同步更新；这些行只读、不可写、不可作为寄存器读取（点“读取选中”会触发一次 `refresh_motor_status`）。
- **实时状态面板（只读）**：勾选「实时刷新」后，每 250ms 调 `refresh_motor_status()`（发 `0xCC` 状态请求帧，**不使能、不会让电机运动**）回读并显示 位置 q / 速度 dq / 力矩 τ / MOS温度 / 线圈温度。轮询任务用 `track_busy=False` 不抢占状态栏，且与读全部/写入互斥（`busy==0` 时才发起）。断开/关闭自动停止。注意：该数值映射依赖 `Limit_Param`(PMAX/VMAX/TMAX)，与电机实际不一致时数值会缩放。这是与 `Gripper_Studio.py` 唯一的“实时”交集，但仅数值、不画图、不控制。
- **参数完整性**：内置寄存器表与 `DM_variable` 严格 1:1（45 个，无缺失/多余/重复）。

- **寄存器表**：内置 45 个 `DM_variable` 全量映射 `(名称, RID, 分组, 读写级别, 单位, 说明)`，按用途分组：设备信息 / 电机特征 / 运动上限 / 运动整定 / 保护阈值 / 控制环 / 通讯·模式 / 工厂校准。
- **读写级别**：`RO`(只读/工厂标定，禁止写) · `RW`(常规可调) · `LIMIT`(运动上限) · `PROT`(保护阈值) · `COMM`(通讯/ID/模式)。写 `LIMIT/PROT/COMM` 在 CLI 需 `--force`，交互菜单需二次确认；写 `RO` 直接拒绝。
- **底层接口**：复用 `CAN.py` 的 `read_motor_param` / `change_motor_param`（0x7FF 写+回读校验）/ `switchControlMode`（RID10）/ `save_motor_param`（先 disable 再 0xAA 写 Flash）。`is_in_ranges` 决定整型(MST_ID/ESC_ID/TIMEOUT/CTRL_MODE/版本/can_br)还是浮点。
- **安全提示**：写 `OV/UV/OC/OT` 时打印说明书建议边界（≤32V / ≥15V / ≤9.8A / 电机≤100℃）；改 `ESC_ID/MST_ID/can_br` 后提示“保存重启后需用新 ID/波特率重连”。
- **用法**：

```powershell
& "C:\Users\KNOW\.conda\envs\xygripper\python.exe" Motor_Param_Tool.py                    # 图形界面(默认, 窗口内填端口)
& "...python.exe" Motor_Param_Tool.py --port COM6     # 图形界面并预填端口
& "...python.exe" Motor_Param_Tool.py repl --port COM6 # 文本交互菜单
& "...python.exe" Motor_Param_Tool.py dump            # 读全部(--all 含工厂校准)
& "...python.exe" Motor_Param_Tool.py get PMAX        # 读单个(名称或 RID)
& "...python.exe" Motor_Param_Tool.py set ACC 5       # 写常规参数
& "...python.exe" Motor_Param_Tool.py set OC_Value 8 --force --save   # 写敏感参数并存盘
& "...python.exe" Motor_Param_Tool.py mode MIT        # 切控制模式
& "...python.exe" Motor_Param_Tool.py save            # 保存到 Flash
& "...python.exe" Motor_Param_Tool.py export p.csv    # 导出 CSV
& "...python.exe" Motor_Param_Tool.py specs           # 仅打印铭牌(无需连接)
```

- **提醒**：`change_motor_param` 仅写入 RAM，必须 `save` 才不丢失；保存会先 disable 电机；改限位/保护阈值后建议重新上电验证（与“硬件/安全注意事项”一致）。

### 碰撞/阻抗/零位 测试台（`Collision_Impedance_Lab.py`）

**定位**：面向三类专项测试的“配置+控制”一体 GUI，区别于 `Gripper_Studio.py`(通用控制+多曲线) 与 `Motor_Param_Tool.py`(寄存器配置)。运行：`python Collision_Impedance_Lab.py`。

- **架构**：复用 `Gripper_Studio` 模式——后台 `LabWorker` 线程独占串口跑 200Hz 阻抗控制环 + 碰撞检测，GUI 主线程读 `snapshot()`/`status()` 并以 `canvas.draw()` 同步刷新(70ms)。
- **① 阻抗测试 (MIT)**：`controlMIT(kp, kd, q_des, 0, tau_ff)`，kp=刚度/kd=阻尼/q_des=平衡位置/前馈力矩/速度(限速平滑)。「保持当前位置」一键把 q_des 设为当前 q。手推电机观察软硬/欠过阻尼。
- **② 碰撞等级测试**：碰撞等级 1~5 映射力矩阈值 `COLLISION_LEVELS={1:3.0,2:2.2,3:1.5,4:1.0,5:0.5}`(Nm，等级越高越灵敏)，可再用滑条微调。`|实测力矩|>阈值` 判定碰撞，上升沿计数(带 0.7×阈值回差去抖)。**碰撞响应三选一**：`报警`(仅标红/计数)、`柔顺让步`(目标跟随当前位置+刚度降至 20%+去前馈，主动让开)、`立即失能`。曲线下图以红点标记碰撞时刻并画 ±阈值线。
- **③ 零位校准**：「设零位」调 `set_zero_position` 把当前位置设为 0（带确认弹窗，提醒坐标系平移），设零后阻抗目标自动对齐 0；「回零位」用阻抗平滑回到 0。
- **保护**：温度超限直接失能；前馈力矩有硬天花板钳制；速度由限速平滑约束。
- **安全**：碰撞检测以 `|tau|` 直接判定，纯位置保持时空载 tau≈0、被外力推/撞时 tau 上升即触发；测试从低刚度、低力矩起步。

### 行程重复定位精度测试台（`Gripper_Repeatability_Lab.py`）

**定位**：专测夹爪在两个行程端点间往返的**重复定位精度 (repeatability)**，区别于其它三个工具。运行：`python Gripper_Repeatability_Lab.py`。

- **测试流程**：后台 `RepeatWorker` 跑位置循环状态机：`前往A→A停留(settle)→记录A到位位置→前往B→B停留→记录B到位位置`，重复 N 次。到位判定 = `|q-目标|<容差 且 |dq|<vel_settle 且 q_slew 到位`（带 `move_timeout` 兜底）。
- **统计**：对 A、B 各 N 个到位样本算 `calc_stats`：样本数 / 均值 / σ(样本标准差) / 极差(max-min) / **重复精度=±极差/2**。GUI 统计面板分 A/B 两列实时刷新。
- **可调参数**（左栏）：行程端点 A/B（滑条或「A=当前/B=当前」抓取当前位置）、速度、刚性 Kp、阻尼 Kd、加持力（合拢端 B 停留时施加的夹持力矩，向合拢方向负力矩；0=纯位置）、重复次数(Spinbox 1~500)、停留时间、到位容差、温度保护。**参数在「开始测试」时冻结快照**（`cfg`），整轮测试条件一致。
- **可视化**：上图=位置-时间轨迹(实测 vs 目标)；下图为**可切换统计图**(右上下拉框)：① 正态分布(到位偏差 pos−均值 的直方图 + 高斯拟合曲线 + ±3σ 虚线，A/B 居中叠加比较离散度)；② 散点趋势(到位 vs 循环 + 均值线)；③ 回归(到位 vs 循环 的一次线性拟合，显示 mrad/次 漂移用于检测系统性偏移)。下图仅在样本数/类型变化时重绘以省开销。
- **导出**：CSV 记录每次 `cycle,endpoint,arrived_pos,target`。
- **安全**：与其它测试台一致——温度超限失能、目标限速平滑、量程钳制、关闭安全收尾；开始测试前确认行程范围不撞击，建议从低刚度/低速起步。

## 开发约定

- 在 `motor/` 目录内运行脚本以满足相对导入。
- 中文输出请设置 `PYTHONIOENCODING=utf-8`。
- 新增控制脚本时复用 `MotorControl` + `MotorComm`，循环内务必调用 `controlMIT`/`control_*` 触发一次 `recv()` 刷新反馈。
- 不要在未连接硬件时盲跑运动脚本；先用 `read_motor_param` 做连通性自检。
