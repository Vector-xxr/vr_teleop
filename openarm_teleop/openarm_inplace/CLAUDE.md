# CLAUDE.md — openarm_inplace C++ Inspace 工作流

本文件为 AI 编码助手及开发者提供当前工程的上下文、约定和实现边界。修改代码前请先阅读本文件和 `README.md`。

## 工程定位

当前目录是 OpenArm Inspace 遥操作的独立工程。目标是让 MuJoCo 仿真和真实 SocketCAN 硬件尽量共用官方 C++ teleop 控制核心，同时把本项目新增/改造代码集中在 `openarm_inplace`，不再污染官方 `openarm_teleop` 子仓库。

不要把当前工作流重新改回纯 Python MuJoCo 控制路径。旧的 `openarm_teleop_inspace` 原型目录已删除；当前实现集中在本目录。官方 `openarm_teleop` 应保持上游状态。

## 核心原则

- 仿真和实机都必须经过官方 C++ `Control` 核心。
- 面板、曲线脚本、CSV 回放都只是参考输入源，不直接承担控制律。
- MuJoCo 只是一个 `TeleopHardware` 后端，用来替代 SocketCAN 进行仿真测试。
- 每条 7 轴机械臂对应一个官方 C++ `Control` 实例，不把左右 14 轴硬塞进一个控制器。
- 双臂仿真使用两个 MuJoCo 世界：leader 世界和 follower 世界。每个世界内左右臂共享同一个 MuJoCo 模型，但左右臂各自由独立 `Control` 对象驱动。
- 仿真 leader/follower 启动 HOME 为所有关节和夹爪 0；不要重新引入 J4=PI/5 的仿真初始姿态。

## 当前 C++ 仿真链路

入口文件：

- `control/openarm_mujoco_panel_control.cpp`

关键对象：

- `MujocoSimWorld`：MuJoCo 模型、窗口、渲染上下文。
- `MujocoTeleopHardware`：把 MuJoCo 关节映射成 `TeleopHardware` 接口。
- `Control`：官方 C++ teleop 控制核心。
- `SimAdminThread`：接收面板、平滑脚本和回放参考。面板/平滑输入写入 leader 和 follower；`--replay` 只写入 follower，leader 保持初始化全 0 HOME。
- `CsvRecorder` / `CsvReplay`：CSV 动作录制与回放。
- `UdpPanelReceiver`：localhost UDP 面板输入。

数据流：

```text
滑块面板 / 平滑脚本 / 回放 CSV
  -> SimAdminThread
  -> RobotSystemState 参考
  -> Control::unilateral_step()
  -> SafetyFilter
  -> joint_to_motor
  -> MITParam
  -> MujocoTeleopHardware
  -> MuJoCo qpos/qvel
```

控制线程每周期调用 `Control::unilateral_step()`。不要绕过这个调用直接从 Python 或 UI 写 MuJoCo 关节作为主要控制路径。

## 双臂数据顺序

双臂逻辑关节顺序统一为 16 路：

```text
left J1..J7, left gripper, right J1..J7, right gripper
```

CSV 头格式：

```text
time,q0,dq0,q1,dq1,...,q15,dq15
```

含义：

- `q0..q7`：左臂 7 关节 + 左夹爪。
- `q8..q15`：右臂 7 关节 + 右夹爪。

UDP 输入兼容两种格式：

- 16 个位置值：`q0,q1,...,q15`。Tk 面板使用此格式，`dq` 会置 0。
- 32 个位置/速度值：`q0,dq0,q1,dq1,...,q15,dq15`。平滑速度测试脚本使用此格式。

CSV 录制始终使用 `q,dq` 成对格式。

MuJoCo 关节映射必须按名字查找：

- `openarm_left_joint1..7`
- `openarm_right_joint1..7`
- `openarm_{left,right}_finger_joint1`
- `openarm_{left,right}_finger_joint2`

不要依赖 MuJoCo `qpos` 的裸数组顺序来推断左右臂或关节编号。

## 录制与回放机制

**核心原则：仿真和真机的录制链路完全一致，都是记录 leader 电机的实际状态（response），而非外部输入的指令（reference）。**

录制时（面板/脚本/send_csv_actions.py 模式）：

```text
UDP 输入 (面板/脚本/send_csv_actions.py)
  -> SimAdminThread external_refs
  -> 设置 leader RobotSystemState references
  -> 读取 leader 实际状态 (get_all_responses)  ← 经过控制器跟踪后的实际位置
  -> follower_refs = leader response
  -> 设置 follower RobotSystemState references
  -> CsvRecorder::append(follower_refs)  ← 录制的是 leader 实际状态
```

真机 Live 模式：

```text
人手动搬动 leader 臂
  -> CAN 读取 leader 电机实际位置 (get_all_responses)
  -> follower_refs = leader response
  -> 设置 follower RobotSystemState references
  -> CsvRecorder::append(follower_refs)  ← 录制的是 leader 实际状态
```

回放时：

```text
CsvReplay::load(path) 或 load_directory(dir_path)
  -> CsvReplay::sample() 按时间戳采样
  -> SimAdminThread external_refs
  -> follower_refs = external_refs (直接使用回放数据，不经过 leader)
  -> 只设置 follower RobotSystemState
  -> follower Control::unilateral_step()
```

因此仿真回放也必须经过 C++ `Control`、安全滤波和硬件后端。回放不是画面播放，也不是绕过控制链路直接写 MuJoCo；它是把 CSV 参考轨迹重新喂给 follower 侧 C++ teleop 控制器。

`send_csv_actions.py` 脚本用于模拟真机主从遥操：读取 recordings 中的动作文件（含衔接文件），通过 UDP 发送给 leader，C++ 端走"leader 实际状态 → follower reference → 录制"的完整链路。

### 目录批量回放

`--replay` 参数支持传入目录路径。`CsvReplay::load_directory()` 会：

1. 扫描目录中所有 `.csv` 文件，跳过衔接文件（`{数字}_{数字}.csv` 格式）。
2. 按文件名开头数字排序加载。
3. 自动检测相邻动作文件之间的关节位置间隙，生成五次多项式衔接轨迹。
4. 衔接文件保存为 `{x}_{a}.csv`（如 `0_1.csv`、`1_2.csv`）。
5. 如果第一个动作起点与零位不同，生成初始衔接 `0_{seq}.csv`。

可通过 `--transition 0` 禁用衔接生成，`--clean-transitions` 一键清理衔接文件。

## 安全与限位

核心文件：

- `src/safety/safety_filter.hpp`
- `src/safety/safety_filter.cpp`

安全滤波负责：

- 非有限值保护。
- 关节和夹爪位置限位。
- 速度限位。
- 首帧跳变限位。
- 实机路径的加速度限位。
- 防止加速度限幅导致越过目标点再反复拉回。

MuJoCo 面板仿真中关闭加速度限幅，以避免滑块输入响应变慢或产生软件滤波导致的目标附近抖动；仍保留位置和速度限位。实机路径应保留更保守的速度/加速度保护。

安全配置函数定义在 `src/common.hpp`：

- `make_default_safety_config(total_joints)`：真机安全配置。
- `make_sim_safety_config(total_joints)`：仿真安全配置。

## 公共基础设施

- `src/common.hpp`：`keep_running` + `signal_handler()`、`make_default_safety_config()`、`make_sim_safety_config()`。三个控制程序共用。
- `src/openarm_constants.hpp`：全局常量（`constexpr`），包括 NJOINTS、NMOTORS、位置/速度/力矩限位。通过 `#define` 别名保持向后兼容。

## 运行命令

构建：

```bash
cmake -S . -B build
cmake --build build -j2
```

双臂 C++ MuJoCo + 面板：

```bash
script/run_cpp_mujoco_with_panel.sh
```

只启动 C++ MuJoCo 进程并录制：

```bash
OPENARM_RECORD_PATH=recordings/smooth_long.csv \
script/run_cpp_mujoco_panel.sh
```

单文件回放：

```bash
script/run_cpp_mujoco_replay.sh recordings/xxx.csv
```

目录批量回放：

```bash
script/run_cpp_mujoco_replay.sh recordings/
```

实机主从直连遥操并录制：

```bash
script/run_cpp_real_teleop_record.sh
```

实机 CSV 回放：

```bash
script/run_cpp_real_replay.sh recordings/real_demo.csv
```

## 实时优先级警告

普通用户下可能看到：

```text
[WARN] Failed to set real-time priority (errno: 1). Try running with sudo or setcap.
```

仿真中可以忽略。实机高频 CAN 遥操时建议设置权限：

```bash
sudo setcap cap_sys_nice+ep build/unilateral_control build/bilateral_control build/mujoco_panel_control
```

## 修改边界

- 优先复用官方 C++ `Control`、`Dynamics`、关节转换器和 MIT 指令结构。
- 不要把主要遥操作逻辑移回 Python。
- 不要让 MuJoCo 仿真和实机形成两套不一致的控制核心。
- UI 或测试脚本只能作为输入源，不能绕开 C++ teleop 核心。
- 真机不使用滑块面板控制；真机录制来自 leader 实机主臂，回放来自 CSV。
- 改动安全滤波时必须同时考虑仿真响应和实机安全。
- 新增录制格式时必须保持 README 中的顺序说明同步更新。
- 修改回放逻辑时保持"只驱动 follower"的语义；不要让 `--replay` 再默认驱动 leader。
- 衔接文件命名格式为 `{数字}_{数字}.csv`，`is_transition_file()` 负责识别，加载目录时自动跳过。
