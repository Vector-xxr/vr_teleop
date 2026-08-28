# openarm_inspace 工程说明

本文件为 AI 编码助手和开发者提供当前工作区的上下文。修改代码前请同时阅读根目录 `README.md` 和 `openarm_inplace/CLAUDE.md`。

## 当前定位

`openarm_inspace` 是一个综合工作区，包含 OpenArm 的官方上游组件和当前 Inspace 遥操作实现。当前活跃开发集中在：

```text
openarm_inplace
```

旧的 `openarm_teleop_inspace` Python-only MuJoCo 原型已经删除。不要重新引入这条旧链路作为主要遥操作方案。官方 `openarm_teleop` 子仓库应保持上游状态，本项目新增/改造代码放在 `openarm_inplace`。

## 推荐工作流

进入当前工程：

```bash
cd /home/lan/openARM_demo/openarm_inspace/openarm_inplace
source /home/lan/miniconda3/etc/profile.d/conda.sh
conda activate openarm_teleop1
```

构建：

```bash
cmake -S . -B build
cmake --build build -j2
```

## 工作区目录

- `openarm_inplace`：当前活跃 C++ Inspace teleop 工程。仿真、实机、录制、回放和滑块输入都应优先在这里维护。
- `openarm_teleop`：官方 C++ teleop 上游子仓库，保持干净，避免直接承载 Inspace 定制改动。
- `openarm_mujoco`：官方 MuJoCo 模型和场景资源。
- `openarm_description`：官方 URDF、xacro 和机器人资产。
- `openarm_can`：官方 SocketCAN/CAN-FD 通信库。
- `openarm_ros2`：官方 ROS 2 集成包。
- `openarm_mujoco_hardware`：ros2_control 的 MuJoCo 硬件接口。
- `openarm`：官方门户和文档网站。
- `arm_control`：额外控制实验代码。

## 核心约定

- MuJoCo 仿真和真实硬件都应尽量复用官方 C++ teleop 控制核心。
- 面板、脚本和 CSV 只是参考输入源，不能替代 C++ 控制链路。
- 仿真后端应作为 `TeleopHardware` 的一种实现，而不是单独写一套 Python 控制器。
- 动作回放应读取 CSV 关节空间参考，再走 follower 侧 C++ 控制链路。
- 当前仿真回放只驱动 follower 端，leader 保持初始化全零 HOME。
- 当前仿真启动时，leader/follower 左右臂所有关节和夹爪都初始化为 0。

## 不建议修改的内容

以下目录主要是官方上游组件，除非任务明确要求，否则不要随意大面积修改：

- `openarm_mujoco`
- `openarm_description`
- `openarm_can`
- `openarm_ros2`
- `openarm`

如果需要调整官方模型、URDF 或 CAN 逻辑，应先确认这个改动确实是当前问题的根因。

## 当前技术路线

当前 `openarm_inplace` 中的主链路为：

```text
滑块面板 / 平滑脚本 / CSV 回放
  -> RobotSystemState reference
  -> Control::unilateral_step()
  -> SafetyFilter
  -> joint_to_motor
  -> MITParam
  -> MujocoTeleopHardware 或 CanTeleopHardware
```

这条链路的重点是最大程度复用官方 C++ teleop 逻辑，使仿真测试更接近实机迁移后的行为。
