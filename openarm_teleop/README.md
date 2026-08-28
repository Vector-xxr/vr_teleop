# openarm_inspace

本目录是当前 OpenArm Inspace 遥操作工作区，包含 OpenArm 的仿真模型、机器人描述、CAN 通信、ROS 2 集成和 C++ 遥操作工程。当前实际维护和推荐使用的遥操作入口是：

```text
openarm_inplace
```

旧的 `openarm_teleop_inspace` Python-only MuJoCo 遥操作原型已经删除。本项目新增和改造的代码已集中到 `openarm_inplace`。官方 `openarm_teleop` 子仓库保持为上游代码，不再承载 Inspace 定制改动。

新增代码的详细技术设计、动作录制/回放机制和安全层说明见：

```text
openarm_inplace/docs/TECHNICAL_DESIGN.md
```

## 当前主链路

进入工程：

```bash
cd /home/lan/openARM_demo/openarm_inspace/openarm_inplace
source /home/lan/miniconda3/etc/profile.d/conda.sh
conda activate openarm_teleop1
```

MuJoCo 仿真和真实 SocketCAN 硬件共用同一条 C++ 控制链路：

```text
滑块面板 / 平滑测试脚本 / 回放 CSV
  -> RobotSystemState reference
  -> 官方 C++ Control::unilateral_step()
  -> SafetyFilter
  -> MITParam 指令生成
  -> MuJoCo 后端或 SocketCAN 后端
```

后端会根据运行模式切换：

- MuJoCo 仿真：`MujocoTeleopHardware`
- 真实硬件：`CanTeleopHardware` + SocketCAN

## 构建

```bash
cd /home/lan/openARM_demo/openarm_inspace/openarm_inplace
cmake -S . -B build
cmake --build build -j2
```

## C++ MuJoCo + 双臂滑块面板

启动两个 MuJoCo 窗口和一个双臂滑块面板：

```bash
cd /home/lan/openARM_demo/openarm_inspace/openarm_inplace
script/run_cpp_mujoco_with_panel.sh
```

会打开：

- leader MuJoCo 窗口
- follower MuJoCo 窗口
- 双臂 Tk 滑块面板

滑块和 UDP 数据顺序统一为：

```text
left J1..J7, left gripper, right J1..J7, right gripper
```

仿真启动时，leader/follower 的左右臂所有关节和夹爪都会初始化为 0。

## 动作录制与回放

录制面板操作：

```bash
cd /home/lan/openARM_demo/openarm_inspace/openarm_inplace
OPENARM_RECORD_PATH=recordings/manual_panel.csv \
script/run_cpp_mujoco_with_panel.sh
```

通过同一条 C++ MuJoCo teleop 链路回放：

```bash
./build/mujoco_panel_control \
  --xml ../openarm_mujoco/v1/scene.xml \
  --urdf ../openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf \
  --replay recordings/manual_panel.csv
```

回放模式下，CSV 轨迹只写入 follower 端。leader 窗口保持初始化的全零 HOME 姿态。因此这里是真正把录制的关节空间参考重新喂给 follower 侧 C++ 控制链路，不是画面假回放。

CSV 格式：

```text
time,q0,dq0,q1,dq1,...,q15,dq15
```

含义：

- `q0..q7`：左臂 7 个关节 + 左夹爪
- `q8..q15`：右臂 7 个关节 + 右夹爪

## 平滑匀速测试

终端 A，启动 C++ MuJoCo 并录制：

```bash
cd /home/lan/openARM_demo/openarm_inspace/openarm_inplace
OPENARM_RECORD_PATH=recordings/smooth_velocity_test.csv \
script/run_cpp_mujoco_panel.sh
```

终端 B，发送大幅度连续关节空间参考，同时输出速度日志：

```bash
cd /home/lan/openARM_demo/openarm_inspace/openarm_inplace
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

该脚本发送 32 路 `q,dq` 交错数据。UDP 接收端也兼容旧的 16 路位置值面板数据。

回放：

```bash
script/run_cpp_mujoco_replay.sh recordings/smooth_velocity_test.csv
```

## 实机面板遥操

实机路径使用 SocketCAN，不会打开 MuJoCo 窗口：

```bash
cd /home/lan/openARM_demo/openarm_inspace/openarm_inplace
script/run_cpp_panel_teleop.sh
```

另开一个终端启动滑块面板：

```bash
script/run_cpp_joint_panel.sh --port 47910
```

常用环境变量：

- `OPENARM_TELEOP_URDF`
- `OPENARM_TELEOP_ARM_SIDE`
- `OPENARM_LEADER_CAN`
- `OPENARM_FOLLOWER_CAN`
- `OPENARM_PANEL_PORT`
- `OPENARM_RECORD_PATH`

## 目录说明

- `openarm_inplace`：当前活跃的 C++ Inspace 遥操作工程，包含 MuJoCo 后端、滑块输入、CSV 录制/回放、平滑轨迹测试和调试工具。
- `openarm_teleop`：官方 C++ teleop 上游子仓库，保持干净，不再直接放置本项目定制改动。
- `openarm_mujoco`：MuJoCo 模型和场景。
- `openarm_description`：URDF 和机器人资产。
- `openarm_can`：SocketCAN/OpenArm CAN 通信支持。
- `openarm_ros2`：ROS 2 集成。
- `arm_control`、`openarm_mujoco_hardware`、`openarm`：上游或实验性组件。

## 实时优先级提示

普通用户运行时可能看到：

```text
[WARN] Failed to set real-time priority (errno: 1). Try running with sudo or setcap.
```

仿真中可以忽略。实机高频 CAN 遥操时建议给可执行文件授予实时调度权限：

```bash
sudo setcap cap_sys_nice+ep \
  openarm_inplace/build/unilateral_control \
  openarm_inplace/build/bilateral_control \
  openarm_inplace/build/mujoco_panel_control
```
