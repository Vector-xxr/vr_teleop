# OpenArm Inspace SOP 操作说明

本文档面向工程机现场操作。后续主工程命令统一从以下目录执行：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
```

当前推荐使用的主工程是 `openarm_inplace`。真机 SocketCAN、双臂录制、真机回放、MuJoCo 仿真、CSV 录制回放都集中在这里。

## 1. 项目定位

`openarm_inspace` 是 OpenArm Inspace 遥操作工作区，主要目录如下：

| 目录 | 说明 |
| --- | --- |
| `openarm_inplace` | 当前主工程，负责真机 SocketCAN、MuJoCo 仿真、录制、回放、滑块输入和安全层。 |
| `openarm_teleop` | 官方 C++ teleop 上游代码，原则上不改。 |
| `openarm_mujoco` | MuJoCo 模型和场景。 |
| `openarm_description` | URDF、xacro 和机器人资产。 |
| `openarm_can` | SocketCAN / CAN-FD 通信支持。 |
| `openarm_ros2` | ROS 2 集成包。 |
| `Gloria-M-SDK` | Gloria/DM 电机实验代码。 |
| `arm_control` | 额外控制实验代码。 |

核心链路：

```text
滑块面板 / 平滑测试脚本 / CSV 回放 / 真机 leader
  -> RobotSystemState reference
  -> C++ Control::unilateral_step()
  -> SafetyFilter
  -> joint_to_motor
  -> MITParam
  -> CanTeleopHardware 或 MujocoTeleopHardware
```

重要原则：

- 当前推荐的真机录制、真机回放和 MuJoCo 面板/回放路径主要走 C++ `Control::unilateral_step()`；旧 `build/bilateral_control` 双边入口仍使用 `Control::bilateral_step()`。
- 面板、CSV、脚本只是 reference 输入源，不直接承担控制律。
- 真机录制写入的是 follower 端 reference；在 Live 主从模式下，这个 reference 来自 leader 实机主臂的 response。
- 真机回放来自 CSV，必须先经过仿真和小幅低速验证。

## 2. 环境准备

每个新终端先执行：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
source ~/miniconda3/etc/profile.d/conda.sh
conda activate openarm_teleop1
```

如果 conda 路径不是 `~/miniconda3`，按工程机实际路径替换。

如果环境不存在：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
conda env create -f environment/openarm_teleop1.environment.yml
conda activate openarm_teleop1
```

系统依赖：

```bash
sudo apt install build-essential cmake pkg-config can-utils
sudo apt install ros-humble-desktop ros-humble-xacro ros-humble-ros2-control ros-humble-ros2-controllers
```

如需 ROS 2：

```bash
source /opt/ros/humble/setup.bash
```

## 3. 编译

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
cmake -S . -B build
cmake --build build -j2
```

主要可执行文件：

| 可执行文件 | 说明 |
| --- | --- |
| `build/unilateral_control` | 真机单边主从遥操作、录制和单臂回放。 |
| `build/dual_replay_control` | 真机双臂合并 CSV 单进程回放。 |
| `build/bilateral_control` | 真机双边遥操作。 |
| `build/mujoco_panel_control` | MuJoCo 仿真、面板输入、CSV 录制和回放。 |
| `build/gravity_comp` | 重力补偿。 |
| `build/comm_test` | 通信测试。 |

## 4. 真机操作前检查

### 4.1 确认 CAN 稳定接口名

工程机现场不要直接依赖 `can0`、`can1`、`can2`、`can3` 这类 Linux 自动枚举名。USB-CAN 在开关机或重新拔插后可能重新排序。

当前代码和脚本推荐使用稳定 SocketCAN 接口名：

```text
右臂 leader   = leader_right
右臂 follower = follower_right
左臂 leader   = leader_left
左臂 follower = follower_left
```

这些名字应由工程机系统层提前绑定到固定 USB-CAN 设备或固定物理口。真机操作命令优先使用这些稳定名字。

说明：

- C++ 代码本身不按 USB 序列号或物理口识别设备。
- C++ 底层只接收一个 SocketCAN interface 名称，例如 `leader_right`。
- 稳定命名应由系统配置完成，代码通过该 interface 名称打开 CAN socket。

### 4.2 检查 CAN 接口

```bash
ip link show type can
ip -details -statistics link show leader_right
ip -details -statistics link show follower_right
ip -details -statistics link show leader_left
ip -details -statistics link show follower_left
```

正常状态应包含：

```text
state UP
can <FD> state ERROR-ACTIVE
berr-counter tx 0 rx 0
bus-off 0
```

如果某一路稳定接口不存在或不是 `UP`，不要启动真机遥操作。

### 4.3 USB-CAN 拔插后恢复

先停掉旧控制程序：

```bash
pkill -f unilateral_control
pkill -f run_cpp_real_teleop_record
pkill -f record_start_panel
```

重新配置四路稳定 CAN-FD 接口：

```bash
sudo ip link set leader_right down
sudo ip link set leader_right type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set leader_right up

sudo ip link set follower_right down
sudo ip link set follower_right type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set follower_right up

sudo ip link set leader_left down
sudo ip link set leader_left type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set leader_left up

sudo ip link set follower_left down
sudo ip link set follower_left type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set follower_left up
```

回到主工程：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
```

再次确认稳定接口：

```bash
ip -details -statistics link show leader_right
ip -details -statistics link show follower_right
ip -details -statistics link show leader_left
ip -details -statistics link show follower_left
```

注意：当前仓库中的 `openarm_can/setup/openarm-can-configure-socketcan-4-arms` 脚本内部写的是 `can0..can3`，不适合作为工程机稳定口名的主恢复命令。工程机应优先使用 `leader_right/follower_right/leader_left/follower_left`。

### 4.4 判断接线是否异常

- 四路 CAN 都有 RX/TX 增长：通常说明双臂都启动。
- 只有某两路 CAN 有增长：可能只有一侧启动。
- leader 被当成 follower：主臂可能变硬、难以推动。
- 左右反接：可能右 leader 驱动左 follower，或左 leader 驱动右 follower。

## 5. 真机双臂录制

### 5.1 启动双臂主从录制

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inspace/openarm_inplace/

script/run_cpp_real_teleop_record_both.sh test_action_01
```

其中 `test_action_01` 是录制的动作轨迹文件名，可替换成其他名称。

启动后流程：

1. 左右臂主从遥操启动。
2. 弹出一个双臂统一录制按钮。
3. 点击按钮后开始写左右 CSV。
4. 动作结束后关闭按钮窗口，或在终端按 `Ctrl+C`。
5. 脚本会停止左右臂，并自动生成合并 CSV。

### 5.2 录制输出

```text
recordings/sessions/test_action_01/
├── real_teleop_split/
│   ├── right/
│   │   └── real_teleop_right.csv
│   └── left/
│       └── real_teleop_left.csv
└── real_teleop_dual_merged.csv
```

说明：

- `real_teleop_split/right/real_teleop_right.csv`：右臂原始录制文件。
- `real_teleop_split/left/real_teleop_left.csv`：左臂原始录制文件。
- `real_teleop_dual_merged.csv`：左右臂重采样到统一时间戳后的 16 路合并文件。

如果不传动作名：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
script/run_cpp_real_teleop_record_both.sh
```

会生成类似：

```text
recordings/sessions/real_YYYYMMDD_HHMMSS/
```

## 6. 真机单臂录制

右臂录制：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace

OPENARM_TELEOP_ARM_SIDE=right_arm \
OPENARM_LEADER_CAN=leader_right \
OPENARM_FOLLOWER_CAN=follower_right \
OPENARM_RECORD_PATH=recordings/right_demo.csv \
script/run_cpp_real_teleop_record.sh
```

左臂录制：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace

OPENARM_TELEOP_ARM_SIDE=left_arm \
OPENARM_LEADER_CAN=leader_left \
OPENARM_FOLLOWER_CAN=follower_left \
OPENARM_RECORD_PATH=recordings/left_demo.csv \
script/run_cpp_real_teleop_record.sh
```

## 7. 真机双臂回放

真机回放前必须完成：

1. MuJoCo 回放验证。
2. 小幅、低速轨迹验证。
3. CAN 映射确认。
4. 工作空间清空。
5. 急停或断电保护准备。

### 7.1 回放单个 session

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace

script/run_cpp_real_replay_both.sh recordings/sessions/test_action_01
```

图形化选择入口：

```bash
bash script/run_cpp_real_replay_panel.sh
```

面板启动后默认进入常驻 `dual_replay_control` 双臂 server；双臂 server 线程创建时先从当前真实姿态平滑回全电机 `0`，再平滑到 `OPENARM_REPLAY_HOME_POSE` 指定的动作准备位。选择 16 关节 merged CSV、文件夹或单臂 split CSV 时都沿用双臂 server；单臂 CSV 会补齐另一臂静止轨迹。按钮可以选择任意直接包含 16 关节 merged CSV 的文件夹、标准 session 文件夹、整个 `recordings/sessions` 根目录、单个 merged CSV，或单臂 split CSV。

面板必须先收到 server `ready` 才会发送动作命令；server 返回 `accepted` 并带回遥测端口后，才会打开对应的力矩面板。关闭面板时会发送 `shutdown`，界面保持响应并等待双臂完成 Kd-only damping release，不默认强制杀掉回放进程。

如果动作需要联动机器人表情，在动作 CSV 表头前写入 `face_emoji` 元数据。回放真正进入该动作段时，面板会异步调用 `face-emoji/app/face.py` 发送表情，不会阻塞机械臂控制：

```text
# face_emoji=happy
time,q0,dq0,tau0,...
```

可选写法：

```text
# face_emoji=happy
# face_emoji=scene:wave
# face_emoji={"type":"expression","intent":"happy","tts":"开始动作"}
```

### 7.2 回放整个 sessions 文件夹

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace

script/run_cpp_real_replay_both.sh recordings/sessions
```

脚本会按 session 文件夹名称排序，准备动作文件并顺序回放。动作之间会自动生成衔接轨迹。

### 7.3 更保守的衔接时间

当前姿态到第一帧的初始衔接默认 `3.0s`；动作间衔接时间默认 `5.0s`。动作间衔接更慢更保守可设置为 `8.0s`：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace

OPENARM_DUAL_REPLAY_TRANSITION_DURATION=8.0 \
script/run_cpp_real_replay_both.sh recordings/sessions/test_action_01
```

### 7.4 真机双臂回放日志

当前默认优先使用 `build/dual_replay_control` 单进程 merged 回放。该模式的主要日志直接输出在启动终端中，不再分别写入右臂/左臂 replay 日志。

只有当输入没有完整 merged 文件、脚本 fallback 到双进程 split 回放时，才会使用以下默认日志：

```text
/tmp/openarm_real_replay_right.log
/tmp/openarm_real_replay_left.log
```

双臂单进程回放时，默认会打开力矩遥测面板，并将数据写到：

```text
recordings/sessions/<session>/torque_telemetry.csv
```

如果直接回放整个 sessions 根目录，例如 `script/run_cpp_real_replay_both.sh recordings/sessions`，默认遥测 CSV 会写到：

```text
recordings/sessions/torque_telemetry.csv
```

通过图形化常驻回放面板启动时，力矩遥测 CSV 默认写到：

```text
recordings/torque_telemetry/<timestamp>_<action>_torque.csv
```

力矩面板显示 16 路反馈力矩、MOS 温度、rotor 温度、`reserved_alert_mask` 兼容保留列和电机故障码；`reserved_alert_mask` 当前固定为 `0`，不再表示心跳或断联状态。

## 8. 真机单臂回放

右臂回放：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace

OPENARM_TELEOP_ARM_SIDE=right_arm \
OPENARM_LEADER_CAN=leader_right \
OPENARM_FOLLOWER_CAN=follower_right \
script/run_cpp_real_replay.sh recordings/sessions/test_action_01/real_teleop_split/right/real_teleop_right.csv
```

左臂回放：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace

OPENARM_TELEOP_ARM_SIDE=left_arm \
OPENARM_LEADER_CAN=leader_left \
OPENARM_FOLLOWER_CAN=follower_left \
script/run_cpp_real_replay.sh recordings/sessions/test_action_01/real_teleop_split/left/real_teleop_left.csv
```

## 9. 真机安全注意事项

### 9.1 回放前检查清单

每次真机回放前执行：

1. 确认 CSV 来自正确 session。
2. 先在 MuJoCo 中回放确认轨迹合理。
3. 确认 CAN 映射仍为 `leader_right/follower_right/leader_left/follower_left`。
4. 检查四路 CAN 都是 `UP`。
5. 使用更长衔接时间进行首次回放。
6. 清空机械臂运动范围。
7. 准备急停或断电。

### 9.2 实时优先级设置

普通用户运行时可能看到：

```text
[WARN] Failed to set real-time priority (errno: 1). Try running with sudo or setcap.
```

仿真中通常可以忽略。实机高频 CAN 遥操建议设置：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
sudo setcap cap_sys_nice+ep \
  build/unilateral_control \
  build/bilateral_control \
  build/mujoco_panel_control \
  build/dual_replay_control
```

## 10. 真机常见问题排查

### 10.1 真机启动后主臂变硬或从臂不跟随

检查：

```bash
ip -details -statistics link show leader_right
ip -details -statistics link show follower_right
ip -details -statistics link show leader_left
ip -details -statistics link show follower_left
```

重点判断：

- CAN 是否 `UP`。
- TX/RX 是否增长。
- leader/follower 是否反接。
- 左右臂是否反接。

当前推荐映射：

```text
右臂 leader   leader_right
右臂 follower follower_right
左臂 leader   leader_left
左臂 follower follower_left
```

### 10.2 真机回放动作异常

处理顺序：

1. 立即急停或停止程序。
2. 检查是否回放了错误 CSV 或错误 session。
3. 检查 CAN 映射是否变化。
4. 回到 MuJoCo 中复现该 CSV。
5. 使用更慢的 `OPENARM_DUAL_REPLAY_TRANSITION_DURATION` 再小幅测试。

## 11. CSV 数据格式

录制 CSV 格式：

```text
time,q0,dq0,q1,dq1,...,q15,dq15
```

关节顺序：

```text
q0..q7   = 左臂 J1..J7 + 左夹爪
q8..q15  = 右臂 J1..J7 + 右夹爪
```

UDP 输入支持两种格式：

```text
16 路位置值：
q0,q1,...,q15
```

```text
32 路位置速度交错值：
q0,dq0,q1,dq1,...,q15,dq15
```

说明：

- 真机录制保存 follower 端 reference；Live 主从模式下该 reference 由 leader 实际 response 生成。
- 真机回放读取 CSV 后作为 follower reference 执行。
- MuJoCo 回放同样只驱动 follower，leader 保持 HOME。

## 12. 常用环境变量

| 环境变量 | 说明 |
| --- | --- |
| `OPENARM_TELEOP_URDF` | 指定 URDF 路径。 |
| `OPENARM_RECORD_PATH` | 指定录制 CSV 路径。 |
| `OPENARM_REPLAY_PATH` | 指定回放 CSV 或目录。 |
| `OPENARM_TELEOP_ARM_SIDE` | 真机单臂侧别，`right_arm` 或 `left_arm`。 |
| `OPENARM_LEADER_CAN` | 单臂 leader CAN 接口。 |
| `OPENARM_FOLLOWER_CAN` | 单臂 follower CAN 接口。 |
| `OPENARM_RIGHT_LEADER_CAN` | 双臂右侧 leader CAN 接口。 |
| `OPENARM_RIGHT_FOLLOWER_CAN` | 双臂右侧 follower CAN 接口。 |
| `OPENARM_LEFT_LEADER_CAN` | 双臂左侧 leader CAN 接口。 |
| `OPENARM_LEFT_FOLLOWER_CAN` | 双臂左侧 follower CAN 接口。 |
| `OPENARM_AUTO_CONFIG_CAN` | 是否自动配置 CAN，默认 `1`。 |
| `OPENARM_SKIP_CAN_CONFIG` | 跳过 CAN 配置，常用于脚本内部。 |
| `OPENARM_SUDO_PASSWORD` | 录制/回放脚本自动配置 CAN 时使用的非交互 sudo 密码；用后脚本会 unset。图形环境下未设置时会弹出密码框。 |
| `OPENARM_MERGED_RECORD_HZ` | 双臂录制合并 CSV 的目标采样率，默认 `1000`。 |
| `OPENARM_DUAL_REPLAY_TRANSITION_DURATION` | 双臂回放动作间衔接时间，默认 `5.0`。 |
| `OPENARM_DUAL_REPLAY_INITIAL_TRANSITION_DURATION` | 双臂回放起始衔接时间，默认 `0.0` 表示自适应。 |
| `OPENARM_DUAL_REPLAY_SPEED` | 双臂单进程回放速度倍率，默认 `1.0`。 |
| `OPENARM_REPLAY_AUTO_HOME` | 双臂回放启动时是否自动回初始位，默认 `1`；设为 `0` 时跳过 `AdjustPosition()`。 |
| `OPENARM_REPLAY_HOME_POSE` | 双臂单进程回放的动作准备位文件，默认 `recordings/sessions/test_zhengbei2/initial_p1.txt`；线程创建和退出的边界位姿仍是全电机 `0`。 |
| `OPENARM_REPLAY_INIT_HOLD_DURATION` | 启动线程时读取当前真实反馈并覆盖旧命令的保持时间，默认 `0.5` 秒。 |
| `OPENARM_REPLAY_STARTUP_GAIN_RAMP_DURATION` | 启动接管时 follower Kp/Kd 从低增益平滑升到正常值的时间，默认 `1.0` 秒。 |
| `OPENARM_REPLAY_STARTUP_GAIN_RAMP_START_SCALE` | 启动接管增益 ramp 的起始比例，默认 `0.1`。 |
| `OPENARM_INIT_DISABLE_BEFORE_ENABLE` | 初始化时是否先 `disable_all()` 再 `enable_all()`，默认 `0`；只有需要清异常状态时才建议设为 `1`。 |
| `OPENARM_REPLAY_COMMAND_PORT` | 常驻回放面板向 `dual_replay_control` server 发送执行命令的 UDP 端口，默认 `47970`。 |
| `OPENARM_REPLAY_STATUS_PORT` | `dual_replay_control` server 向常驻回放面板回传状态的 UDP 端口，默认 `47971`。 |
| `OPENARM_REPLAY_SHUTDOWN_WAIT` | 重启常驻回放 server 时等待优雅退出的时间；关闭面板时默认等待 damping release 完成，不强制杀进程。 |
| `OPENARM_FACE_EMOJI_ENABLED` | 是否启用动作开始时的 face-emoji 联动，默认 `1`；设为 `0` 关闭。 |
| `OPENARM_FACE_EMOJI_HOST` | face-emoji TCP 服务地址，默认 `192.168.2.199`。 |
| `OPENARM_FACE_EMOJI_PORT` | face-emoji TCP 服务端口，默认 `22345`。 |
| `OPENARM_FACE_EMOJI_TIMEOUT` | face-emoji 连接/读取超时，默认 `5.0` 秒。 |
| `OPENARM_REPLAY_SYNC_START_DELAY` | 双进程 fallback 回放同步 start 前等待时间，默认 `1.5` 秒。 |
| `OPENARM_TORQUE_PANEL` | 是否自动打开真机双臂回放力矩面板，默认 `1`。 |
| `OPENARM_TORQUE_TELEMETRY_PORT` | 力矩遥测 UDP 端口，默认 `47940`。 |
| `OPENARM_TORQUE_PANEL_BASE_PORT` | 常驻回放面板在 server `accepted` 后为每次新开的力矩面板分配端口的起始值，默认 `47980`。 |
| `OPENARM_TORQUE_TELEMETRY_OUTPUT` | 力矩遥测 CSV 输出路径；未设置时写到 replay 目录下。 |
| `OPENARM_TORQUE_RECOVERY_SPEED` | 真实反馈力矩/速度超限后的安全恢复速度，默认 `0.25 rad/s`。 |
| `OPENARM_TORQUE_RECOVERY_MIN_DURATION` | 安全恢复最短平滑时间，默认 `3.0` 秒。 |
| `OPENARM_TELEOP_MONITOR_PANEL` | 真机双臂录制时是否自动打开 leader/follower 数据监控面板，默认 `1`。 |
| `OPENARM_RIGHT_TELEOP_MONITOR_PORT` / `OPENARM_LEFT_TELEOP_MONITOR_PORT` | 左右臂监控 UDP 端口，默认 `47950/47951`。 |
| `OPENARM_TELEOP_AUTO_HOME` | 真机录制启动时是否自动回初始位，默认 `1`；设为 `0` 时保持当前零位/当前位置。 |
| `OPENARM_TELEOP_LIVE_WAIT_FOR_START` | 真机录制启动后是否等待点击 Start 再接入 leader，默认 `0`，即启动后立即 live 跟随。 |
| `OPENARM_TELEOP_LIVE_RAMP_DURATION` | 真机 live 主从接入时 follower 平滑接入时间，默认 `2.0` 秒。 |
| `OPENARM_TELEOP_LIVE_VELOCITY_SCALE` | live 模式 leader 速度前馈缩放，默认 `0.0`。 |
| `OPENARM_GRIPPER_CONTACT_HOLD` | 是否启用夹爪接触保持保护，默认 `0`；主从遥操优先保持正常位置/力矩跟随。 |
| `OPENARM_GRIPPER_CONTACT_EFFORT` | 夹爪接触保持触发力矩阈值，默认 `0.30 Nm`。 |
| `OPENARM_GRIPPER_STALL_VELOCITY` | 夹爪堵转速度阈值，默认 `0.08 rad/s`。 |
| `OPENARM_GRIPPER_CONTACT_POSITION_ERROR` | 夹爪目标位置继续压向反馈位置的最小误差，默认 `0.01 rad`。 |
| `OPENARM_GRIPPER_CONTACT_HOLD_TIME` | 连续满足接触条件后进入保持的时间，默认 `0.20` 秒。 |
| `OPENARM_GRIPPER_RELEASE_DELTA` | leader/轨迹目标张开超过保持位置的释放阈值，默认 `0.06 rad`。 |
| `OPENARM_GRIPPER_HOLD_KP_SCALE` | 夹爪保持模式下 Kp 比例，默认 `0.8`。 |
| `OPENARM_GRIPPER_SLIP_BOOST` | 是否启用夹爪防滑增强，默认 `1`。 |
| `OPENARM_GRIPPER_SLIP_BOOST_KP_SCALE` | 检测到夹爪被外力撑开时短时提升的 Kp 比例，默认 `1.15`。 |
| `OPENARM_GRIPPER_SLIP_BOOST_POSITION_DELTA` | 保持状态下检测到被撑开的位移阈值，默认 `0.012 rad`。 |
| `OPENARM_GRIPPER_SLIP_BOOST_VELOCITY` | 保持状态下检测到被撑开的打开速度阈值，默认 `0.04 rad/s`。 |
| `OPENARM_GRIPPER_SLIP_BOOST_HOLD_TIME` | 防滑增强触发后至少保持的时间，默认 `0.25` 秒。 |
| 夹爪力矩记录 | 主从遥操会保存 leader 夹爪 `tau` 到录制 CSV，但当前控制输出不使用夹爪力矩前馈；follower 夹爪只做位置跟随。 |
| `OPENARM_GRIPPER_CLOSE_DIRECTION` | 夹爪闭合方向，默认 `0` 自动判断；若现场方向明确，可设为 `1` 或 `-1`。 |
| `OPENARM_ADJUST_POSITION_SPEED` | 启动自动回初始位的最大平滑速度，默认 `0.25 rad/s`。 |
| `OPENARM_ADJUST_POSITION_MIN_DURATION` | 启动自动回初始位的最短平滑时间，默认 `4.0` 秒。 |
| `OPENARM_PANEL_PORT` | MuJoCo 面板 UDP 端口，默认 `47910`。 |
| `OPENARM_REPLAY_PANEL_SPEED` | 常驻动作回放面板速度框初始值，默认 `1.0`。 |
| `OPENARM_MUJOCO_XML` | 指定 MuJoCo XML 场景路径。 |
| `OPENARM_DEBUG_CSV` | 指定 debug CSV 输出路径。 |

## 13. MuJoCo 仿真操作

仿真用于真机前验证，也可用于录制测试动作。

### 13.1 启动双臂 MuJoCo + 滑块面板

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
script/run_cpp_mujoco_with_panel.sh
```

该命令会打开：

- leader MuJoCo 窗口
- follower MuJoCo 窗口
- 双臂 Tk 滑块面板

默认录制文件：

```text
recordings/mujoco_panel.csv
```

自定义录制路径：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
OPENARM_RECORD_PATH=recordings/manual_panel.csv \
script/run_cpp_mujoco_with_panel.sh
```

### 13.2 只启动 MuJoCo 控制进程

适合配合脚本发送轨迹：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
OPENARM_RECORD_PATH=recordings/smooth_velocity_test.csv \
script/run_cpp_mujoco_panel.sh
```

### 13.3 单独启动滑块面板

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
script/run_cpp_joint_panel.sh --port 47910 --arms bimanual
```

## 14. MuJoCo 平滑轨迹测试

终端 A，启动 MuJoCo 并录制：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
OPENARM_RECORD_PATH=recordings/smooth_velocity_test.csv \
script/run_cpp_mujoco_panel.sh
```

终端 B，发送双臂连续关节空间参考：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
python script/send_smooth_joint_motion.py \
  --port 47910 \
  --side both \
  --profile constant \
  --duration 30 \
  --cycles 2 \
  --amplitude 0.75 \
  --hz 500 \
  --velocity-log recordings/smooth_velocity_command.csv
```

注意：

- `send_smooth_joint_motion.py` 发送 32 路 `q,dq` 交错数据。
- 滑块面板发送 16 路位置数据。
- 运行平滑脚本时不要同时打开滑块面板，否则面板可能覆盖脚本输入。

## 15. MuJoCo 动作录制

面板录制：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
OPENARM_RECORD_PATH=recordings/manual_panel.csv \
script/run_cpp_mujoco_with_panel.sh
```

直接使用二进制录制：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
./build/mujoco_panel_control \
  --xml ../openarm_mujoco/v1/scene.xml \
  --urdf ../openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf \
  --record recordings/manual_panel.csv
```

CSV 动作发送给 leader 后再录制，终端 A：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
./build/mujoco_panel_control \
  --xml ../openarm_mujoco/v1/scene.xml \
  --urdf ../openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf \
  --record recordings/re_recorded.csv
```

终端 B：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
python script/send_csv_actions.py recordings/ --port 47910
```

## 16. MuJoCo 动作回放

单文件回放：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
script/run_cpp_mujoco_replay.sh recordings/manual_panel.csv
```

目录批量回放：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
script/run_cpp_mujoco_replay.sh recordings/
```

目录回放规则：

- 扫描目录中的 `.csv` 文件。
- 跳过 `{数字}_{数字}.csv` 形式的衔接文件。
- 按文件名开头数字排序。
- 自动生成动作之间的平滑衔接轨迹。

重复 5 次：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
script/run_cpp_mujoco_replay.sh recordings/manual_panel.csv --repeat 5
```

无限循环：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
script/run_cpp_mujoco_replay.sh recordings/manual_panel.csv --loop
```

禁用衔接：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
script/run_cpp_mujoco_replay.sh recordings/ --transition 0
```

清理衔接文件：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
./build/mujoco_panel_control \
  --xml ../openarm_mujoco/v1/scene.xml \
  --urdf ../openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf \
  --replay recordings/ \
  --clean-transitions
```

## 17. MuJoCo 常见问题

### 17.1 回放窗口一闪而过

检查：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
ls -lh recordings/xxx.csv
```

常见原因：

- CSV 路径不对。
- CSV 是空文件。
- CSV 最后一行被强杀截断，有效帧太少。
- 从旧目录或错误目录运行了残留二进制。
- MuJoCo / GLFW 环境异常。

建议使用脚本启动：

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
script/run_cpp_mujoco_replay.sh recordings/xxx.csv
```

正常日志应能看到类似：

```text
[replay] loaded N frames from xxx.csv
[replay] total T frames from N file(s), duration Ds
```

## 18. 推荐现场工作流

### 18.1 录制一个真机双臂动作

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
source ~/miniconda3/etc/profile.d/conda.sh
conda activate openarm_teleop1
cmake -S . -B build
cmake --build build -j2

script/run_cpp_real_teleop_record_both.sh action_name
```

### 18.2 先仿真验证录制动作

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace
script/run_cpp_mujoco_replay.sh recordings/sessions/action_name/real_teleop_dual_merged.csv
```

### 18.3 再真机保守回放

```bash
cd ~/Desktop/tianxiang/openarm_inspace/openarm_inplace

OPENARM_DUAL_REPLAY_TRANSITION_DURATION=8.0 \
script/run_cpp_real_replay_both.sh recordings/sessions/action_name
```

## 19. 修改边界

- 新增和定制代码优先放在 `openarm_inplace`。
- 不要随意大面积修改官方上游目录，如 `openarm_teleop`、`openarm_mujoco`、`openarm_description`、`openarm_can`、`openarm_ros2`。
- 修改安全滤波时必须同时考虑仿真响应和实机安全。
- 修改录制格式时必须同步更新 README 和本 SOP。
- 修改回放逻辑时保持 follower-only 回放语义。
- 真机不使用滑块面板控制；真机录制写入 follower reference，回放来自 CSV。
