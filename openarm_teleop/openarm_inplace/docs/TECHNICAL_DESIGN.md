# openarm_inplace 技术设计说明

本文档单独总结 `openarm_inplace` 中相对官方 `openarm_teleop` 新增或改造的代码逻辑。重点说明动作录制、动作回放和安全层；其他模块按功能做总体说明。

## 1. 工程目标

`openarm_inplace` 的目标是把本项目新增的 Inspace 遥操作能力从官方 `openarm_teleop` 子仓库中独立出来，同时继续最大程度复用官方 C++ teleop 控制核心。

核心原则：

- MuJoCo 仿真和真实 SocketCAN 硬件都走 C++ `Control::unilateral_step()`。
- 滑块面板、平滑测试脚本和 CSV 回放只作为 reference 输入源，不直接承担控制律。
- MuJoCo 被封装为一个硬件后端，替代真实 CAN 硬件参与同一条控制链路。
- 动作录制记录关节空间 reference，不记录画面、不记录 MuJoCo 内部状态、不记录 CAN 原始帧。
- 动作回放读取录制 CSV 后，只写入 follower 端 reference，再经过 C++ 控制链路生成动作。

整体链路：

```text
滑块面板 / 平滑脚本 / CSV 回放
  -> SimAdminThread
  -> RobotSystemState reference
  -> Control::unilateral_step()
  -> SafetyFilter
  -> joint_to_motor
  -> MITParam
  -> MujocoTeleopHardware 或 CanTeleopHardware
```

## 2. 新增和改造代码概览

### 2.1 C++ 仿真入口

文件：

```text
control/openarm_mujoco_panel_control.cpp
```

功能：

- 启动两个 MuJoCo world：leader window、follower window。
- 每个 world 中包含左右臂。
- 每条 7 轴机械臂单独创建一个 C++ `Control` 实例。
- 支持三类输入：UDP 滑块面板、UDP 平滑脚本、CSV replay。
- 支持 CSV 录制和 debug CSV。
- 支持频率统计：control loop Hz、backend send Hz、render Hz。

重要线程：

- `SimControlThread`：周期调用 `Control::unilateral_step()`。
- `SimRenderThread`：周期渲染 MuJoCo 窗口。
- `SimAdminThread`：接收 UDP 或 CSV replay reference，并写入 `RobotSystemState`。

### 2.2 硬件抽象层

文件：

```text
src/hardware/teleop_hardware.hpp
src/hardware/teleop_hardware.cpp
src/hardware/mujoco_teleop_hardware.hpp
src/hardware/mujoco_teleop_hardware.cpp
```

功能：

- `TeleopHardware` 定义统一硬件接口。
- `CanTeleopHardware` 封装官方 `openarm::can::socket::OpenArm`，用于真实 SocketCAN。
- `MujocoTeleopHardware` 封装 MuJoCo 模型，模拟同样的硬件读写接口。

### 2.3 IO 层

文件：

```text
src/io/teleop_io.hpp
src/io/teleop_io.cpp
src/common.hpp
```

功能：

- `UdpPanelReceiver`：接收 localhost UDP reference。
- `UdpTextReceiver`：接收 localhost UDP 文本消息（如录制开始信号）。
- `CsvRecorder`：写入动作录制 CSV。
- `CsvReplay`：读取并按时间戳采样 CSV，支持单文件回放和目录批量回放。
- `common.hpp`：公共信号处理、安全配置函数。

### 2.4 安全层

文件：

```text
src/safety/safety_filter.hpp
src/safety/safety_filter.cpp
```

功能：非有限值保护、位置限位、速度限位、首帧跳变限位、加速度限位、安全停止模式。

### 2.5 Python 脚本

文件：

```text
script/cpp_joint_panel.py
script/send_smooth_joint_motion.py
script/send_csv_actions.py
```

功能：

- `cpp_joint_panel.py`：Tk 双臂滑块面板，发送 16 路位置值。
- `send_smooth_joint_motion.py`：发送连续关节空间测试轨迹，默认发送 32 路 `q,dq` 交错值。
- `send_csv_actions.py`：读取 recordings 目录中的动作轨迹文件，通过 UDP 发送给 leader，模拟真机主从遥操录制链路。

### 2.6 启动脚本

文件：

```text
script/run_cpp_mujoco_panel.sh
script/run_cpp_mujoco_with_panel.sh
script/run_cpp_mujoco_replay.sh
script/run_cpp_real_teleop_record.sh
script/run_cpp_real_replay.sh
script/run_cpp_joint_panel.sh
```

## 3. 动作录制机制

### 3.1 录制目标

录制保存的是双臂关节空间 reference：

```text
time,q0,dq0,q1,dq1,...,q15,dq15
```

共 16 路逻辑关节：

```text
left J1..J7, left gripper, right J1..J7, right gripper
```

对应含义：

- `q0..q7`：左臂 7 个关节 + 左夹爪。
- `q8..q15`：右臂 7 个关节 + 右夹爪。
- 每个 `q` 后面紧跟对应 `dq`。

### 3.2 录制入口

MuJoCo 录制（面板/平滑脚本输入）：

```bash
./build/mujoco_panel_control \
  --xml ../openarm_mujoco/v1/scene.xml \
  --urdf ../openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf \
  --record recordings/manual_panel.csv
```

MuJoCo 录制（CSV 动作文件发送给 leader，模拟真机主从遥操）：

```bash
# 终端1：启动仿真（带录制）
./build/mujoco_panel_control \
  --xml ../openarm_mujoco/v1/scene.xml \
  --urdf ../openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf \
  --record recordings/re_recorded.csv

# 终端2：发送动作文件给 leader
python script/send_csv_actions.py recordings/ --port 47910
```

真机主从遥操录制：

```bash
script/run_cpp_real_teleop_record.sh
```

### 3.3 录制数据来源与链路

**核心原则：仿真和真机的录制链路完全一致，都是记录 leader 电机的实际状态（response），而非外部输入的指令（reference）。**

录制发生在 `SimAdminThread::on_timer()`（仿真）或 `AdminThread::on_timer()`（真机）中。

**仿真面板/脚本模式**（与真机 Live 模式一致）：

```text
UDP 输入 (面板/脚本/send_csv_actions.py)
  -> external_refs
  -> set leader references (leader 跟踪外部输入)
  -> read leader actual state (get_all_responses)  ← 经过控制器跟踪后的实际位置
  -> follower_refs = leader response
  -> set follower references
  -> CsvRecorder::append(follower_refs)  ← 录制的是 leader 实际状态
```

**真机 Live 模式**：

```text
人手动搬动 leader 臂
  -> CAN 读取 leader 电机实际位置 (get_all_responses)
  -> follower_refs = leader response
  -> set follower references
  -> CsvRecorder::append(follower_refs)  ← 录制的是 leader 实际状态
```

**仿真回放模式**（replay 直接驱动 follower，不经过 leader）：

```text
CsvReplay::sample()
  -> external_refs
  -> follower_refs = external_refs (直接使用回放数据)
  -> set follower references only
  -> CsvRecorder::append(follower_refs)  如果同时指定了 --record
```

两种录制模式的区别：

| 场景 | follower reference 来源 | 录制内容 |
|------|------------------------|---------|
| 面板/脚本/send_csv_actions | leader 实际状态 (response) | leader 经过控制器跟踪后的位置 |
| 真机 Live | leader 电机实际位置 (response) | 人操作 leader 的真实轨迹 |
| 回放 + --record | 回放数据 (reference) | 回放的原始指令 |

面板/脚本模式和真机 Live 模式的录制链路完全一致：都是先驱动 leader，再从 leader 的实际状态读取作为 follower reference 并录制。因此仿真录制的动作在真机上回放时，行为是一致的。

### 3.4 UDP 输入格式

`UdpPanelReceiver` 兼容两种格式。

16 路位置值：

```text
q0,q1,...,q15
```

使用场景：Tk 滑块面板。速度 `dq` 自动置 0。

32 路位置/速度交错值：

```text
q0,dq0,q1,dq1,...,q15,dq15
```

使用场景：`send_smooth_joint_motion.py`。录制文件中会保留脚本给出的速度 reference。

### 3.5 CsvRecorder 实现细节

文件：`src/io/teleop_io.cpp`

**`CsvRecorder::open(path, joint_count)`**

1. 打开输出文件流。
2. 记录 `start_ = steady_clock::now()` 作为时间零点。
3. 写入 CSV header：`time,q0,dq0,q1,dq1,...,qN,dqN`。

**`CsvRecorder::append(joints)`**

1. 计算相对时间 `t = now - start_`（秒）。
2. 顺序写入 `t`，然后每个关节的 `position` 和 `velocity`。
3. 每次调用写一行。

**`CsvRecorder::close()`**

关闭文件流。

注意：

- 录制时间戳来自 C++ 录制端，不来自 Python 脚本。
- 如果强制 `kill -9` 或直接杀进程，最后一行 CSV 可能来不及 flush 完整。正常 Ctrl+C 或关闭窗口更稳。
- 高速 debug CSV 会影响时序，普通动作录制不建议同时打开大量 debug。

## 4. 动作回放机制

### 4.1 回放目标

回放不是播放画面，也不是直接把 MuJoCo 关节动画写出来。当前回放是：

```text
读取 CSV 关节空间 reference
  -> 按时间戳采样
  -> 只写入 follower RobotSystemState reference
  -> follower Control::unilateral_step()
  -> SafetyFilter
  -> MITParam
  -> MujocoTeleopHardware 或 CanTeleopHardware
```

因此回放仍然经过 C++ 控制核心和安全层。

### 4.2 回放命令

单文件回放：

```bash
script/run_cpp_mujoco_replay.sh recordings/xxx.csv
```

目录批量回放（按文件名开头数字排序）：

```bash
script/run_cpp_mujoco_replay.sh recordings/
```

重复执行 N 遍后自动退出：

```bash
script/run_cpp_mujoco_replay.sh recordings/xxx.csv --repeat 5
```

无限循环：

```bash
script/run_cpp_mujoco_replay.sh recordings/xxx.csv --loop
```

真机回放：

```bash
script/run_cpp_real_replay.sh recordings/xxx.csv
```

### 4.3 CLI 参数

仿真回放（`mujoco_panel_control`）：

| 参数 | 说明 |
|------|------|
| `--replay PATH` | 回放 CSV 文件或目录 |
| `--transition 0\|1` | 是否生成衔接轨迹文件（默认 1） |
| `--clean-transitions` | 删除目录中所有衔接文件 |
| `--loop` | 无限循环回放 |
| `--repeat N` | 重复回放 N 次 |
| `--record CSV` | 回放同时录制到新 CSV |
| `--debug-csv CSV` | 输出调试 CSV |

真机回放（`unilateral_control`）：

| 参数 | 说明 |
|------|------|
| `--replay PATH` | 回放 CSV 文件或目录 |
| `--transition 0\|1` | 是否生成衔接轨迹文件（默认 1） |
| `--clean-transitions` | 删除目录中所有衔接文件 |
| `--reference-source replay` | 设置参考源为回放 |
| `--no-auto-home` | 跳过启动时 AdjustPosition |

### 4.4 CsvReplay 单文件回放实现

文件：`src/io/teleop_io.cpp`

**加载阶段：`CsvReplay::load(path, joint_count)`**

1. 调用 `load_frames_from_csv()` 解析 CSV 文件。
2. `load_frames_from_csv()` 的具体流程：
   - 打开文件，跳过 header 行。
   - 逐行读取，按逗号分割，`stod` 转换为 double。
   - 验证每行字段数为 `1 + 2 * joint_count`（时间 + 每关节 position/velocity）。
   - 构造 `Frame {t, joints}`，其中 `joints[i] = {position, velocity, effort=0}`。
   - 跳过格式不正确的行。
3. 记录 `start_ = steady_clock::now()`，`index_ = 0`。

**采样阶段：`CsvReplay::sample()`**

1. 计算当前相对时间 `t = now - start_`。
2. `index_` 向前推进到最后一个 `frame.t <= t` 的帧（单调递进，不回退）。
3. 返回该帧的 `joints` 向量。
4. 如果所有帧都已播放完毕，返回 `std::nullopt`。

当前采样策略是零阶保持：

- 不做插值。
- 两个 CSV 帧之间保持上一帧 reference。
- 如果 CSV 频率足够高（如 200Hz+），视觉上是连续的。

**重置：`CsvReplay::reset()`**

重置 `start_` 和 `index_`，用于循环回放。

**结束判断：`CsvReplay::finished()`**

返回 `index_ + 1 >= frames_.size()`。

### 4.5 CsvReplay 目录批量回放实现

**`CsvReplay::load_directory(dir_path, joint_count, generate_transitions)`**

这是目录回放的核心函数，支持按顺序加载多个动作文件并自动拼接。

**步骤 1：扫描与排序**

1. 遍历目录下所有 `.csv` 文件。
2. 跳过衔接文件（匹配 `{数字}_{数字}.csv` 模式的文件，如 `0_1.csv`、`1_2.csv`）。
3. 提取每个文件名开头的数字作为排序序号（如 `1hello.csv` → 1，`2bezier.csv` → 2）。
4. 按序号数值排序，确保 `1_xxx` → `2_xxx` → `10_xxx` 的正确顺序。

**步骤 2：逐文件加载**

对每个排序后的 CSV 文件调用 `load_frames_from_csv()` 加载帧数据，保存文件路径和序号。

**步骤 3：生成初始衔接轨迹**

如果 `generate_transitions == true` 且第一个动作文件的起点与零位姿态不同：

1. 构造零位姿态：所有关节 `position=0, velocity=0, effort=0`。
2. 终点速度设为 0（机器人从静止启动）。
3. 调用 `generate_transition()` 生成衔接帧。
4. 保存为 `0_{first_seq}.csv`（如 `0_1.csv`）。
5. 衔接帧插入到 `frames_` 最前面，时间偏移累加。

**步骤 4：逐文件拼接与动作间衔接**

对每个动作文件：

1. 如果不是第一个文件且 `generate_transitions == true`：
   - 取前一个动作的最后一帧作为衔接起点（含速度）。
   - 取当前动作的第一帧作为衔接终点（含速度）。
   - 调用 `generate_transition()` 生成衔接帧。
   - 保存为 `{prev_seq}_{curr_seq}.csv`（如 `1_2.csv`）。
   - 衔接帧插入 `frames_`，时间偏移累加。
2. 将当前动作的所有帧加上时间偏移后追加到 `frames_`。

**步骤 5：完成**

记录 `start_` 和 `index_`，输出总帧数和总时长。

### 4.6 衔接轨迹生成算法

**`CsvReplay::generate_transition(start_joints, end_joints, time_offset, joint_count)`**

当相邻动作文件之间的关节位置存在间隙时，自动生成平滑衔接轨迹。

**生成条件**：起点和终点的最大位置差 > 1e-6 rad。如果位置完全一致，不生成衔接。

**五次多项式插值**：

```text
q(τ) = p₀ + v₀·T·τ + a₃·τ³ + a₄·τ⁴ + a₅·τ⁵
```

其中 τ ∈ [0, 1]，T 为总时长。

6 个边界条件：

| 条件 | 公式 | 含义 |
|------|------|------|
| q(0) = p₀ | 起点位置 | 匹配前一动作终点位置 |
| q(1) = p₁ | 终点位置 | 匹配后一动作起点位置 |
| q'(0) = v₀·T | 起点速度 | 匹配前一动作终点速度 |
| q'(1) = v₁·T | 终点速度 | 匹配后一动作起点速度 |
| q''(0) = 0 | 起点加速度为零 | 无加速度突变 |
| q''(1) = 0 | 终点加速度为零 | 无加速度突变 |

求解系数：

```text
a₃ = 10·Δp - 6·v₀·T - 4·v₁·T
a₄ = -15·Δp + 8·v₀·T + 7·v₁·T
a₅ = 6·Δp - 3·(v₀ + v₁)·T
```

其中 Δp = p₁ - p₀。

**速度计算**：

```text
q'(τ) = v₀ + (3·a₃·τ² + 4·a₄·τ³ + 5·a₅·τ⁴) / T
```

**电机运动最少**：只有位置不同的关节会运动。如果某关节 Δp ≈ 0，则该关节在衔接过程中保持不动（velocity ≈ 0）。

**自适应时长计算**：

```text
T = max(最大位移 / v_ref, 最大边界速度 / a_ref, 0.5s)
T = min(T, 5.0s)
```

- `v_ref = 1.0 rad/s`：参考速度。
- `a_ref = 2.0 rad/s²`：参考加速度。
- 最短 0.5s，最长 5.0s。

**采样率**：1000Hz（dt = 0.001s）。

**初始衔接的特殊处理**：零位 → 第一个动作的衔接，终点速度强制为 0（机器人从静止启动）。

### 4.7 衔接文件命名规则

衔接文件保存在 recordings 目录中，命名格式为 `{x}_{a}.csv`：

- `x`：前一个动作文件的序号。
- `a`：后一个动作文件的序号。

示例：

```text
0_1.csv       零位 → 动作1 的衔接
1_action.csv  动作1（原始录制文件）
1_2.csv       动作1 → 动作2 的衔接
2_action.csv  动作2（原始录制文件）
2_3.csv       动作2 → 动作3 的衔接
3_action.csv  动作3（原始录制文件）
```

衔接文件的时间戳从 0 开始，与加载后的全局时间无关。

**识别规则**：`is_transition_file()` 匹配 `{数字}_{数字}.csv` 模式。加载目录时自动跳过这些文件，避免重复处理。

### 4.8 衔接文件管理

**生成控制**：

```bash
# 生成衔接文件（默认行为）
--replay recordings/ --transition 1

# 不生成衔接文件，只按顺序回放原始动作
--replay recordings/ --transition 0
```

**一键清理**：

```bash
# 删除目录中所有衔接文件
--replay recordings/ --clean-transitions
```

`CsvReplay::clean_transitions(dir_path)` 会扫描目录，删除所有匹配衔接文件命名模式的 `.csv` 文件。

### 4.9 follower-only 回放

在 `SimAdminThread::on_timer()` 中有明确区分：

```text
if (replay_ == nullptr) {
    leader_left_state_->set_all_references(left_refs);
    leader_right_state_->set_all_references(right_refs);
}
follower_left_state_->set_all_references(left_refs);
follower_right_state_->set_all_references(right_refs);
```

含义：

- 面板/平滑脚本模式：leader 和 follower 都显示同样 reference，便于观察。
- replay 模式：只驱动 follower，leader 保持初始化 HOME。

这是为了让回放符合"录制动作在从端执行"的语义，而不是让 leader 也跟着 replay 动。

### 4.10 回放常见问题

窗口刚开就关闭，常见原因：

- 从旧的 `openarm_teleop` 目录运行了残留二进制。
- CSV 路径不对。
- CSV 是空文件。
- CSV 最后一行被强杀截断，虽然当前解析会跳过坏行，但如果有效帧太少，回放看起来会很快结束。
- MuJoCo/GLFW 环境报错。

建议使用脚本启动，它会打印关键日志：

```text
[replay] loaded N frames from xxx.csv (seq=1)
[replay] initial transition M frames (X.Xs) -> 1xxx.csv
[replay] saved transition file: recordings/0_1.csv
[replay] transition K frames (Y.Ys) 1xxx.csv -> 2xxx.csv
[replay] saved transition file: recordings/1_2.csv
[replay] total T frames from N file(s), duration Ds
```

如果能看到 `loaded N frames` 且频率日志持续打印，说明回放链路是通的。

## 5. 安全层设计

### 5.1 SafetyFilter 输入输出

文件：

```text
src/safety/safety_filter.hpp
src/safety/safety_filter.cpp
```

接口：

```cpp
SafetyStatus filter(
    const std::vector<JointState>& raw_cmd,
    const std::vector<JointState>& current_state,
    double dt,
    std::vector<JointState>* safe_cmd);
```

### 5.2 SafetyConfig

```cpp
struct SafetyConfig {
    std::vector<double> min_position;
    std::vector<double> max_position;
    std::vector<double> max_velocity;
    std::vector<double> max_acceleration;
    double max_first_step_delta = 0.35;
};
```

### 5.3 过滤步骤

1. 检查输出指针是否为空。
2. 如果处于 `SafetyMode::Stop`：输出当前状态，所有关节速度和力矩置 0，返回 emergency stop。
3. 检查输入 command 和 current_state 尺寸是否一致。
4. 如果尚未初始化，用当前状态初始化 `last_cmd_`。
5. 对每个关节执行：非有限值检查、位置限位、首帧跳变限位、速度限位、加速度限位。
6. 把 `safe_cmd` 保存为下一周期的 `last_cmd_`。

### 5.4 仿真和实机的差异

MuJoCo 仿真入口使用 `make_sim_safety_config()`：

```cpp
config.max_velocity[i] = std::max(velocity_limit[idx], 12.0);  // 仿真允许更高速度
config.max_acceleration[i] = 0.0;                                // 仿真关闭加速度限位
config.max_first_step_delta = 2.0;                               // 仿真允许较大首帧跳变
```

实机路径使用 `make_default_safety_config()`：

```cpp
config.max_velocity[i] = std::min(velocity_limit[idx], 2.0);   // 实机限制速度上限
config.max_acceleration[i] = 8.0;                                // 实机保留加速度限位
```

这两个函数定义在 `src/common.hpp` 中。

## 6. MuJoCo 后端实现

### 6.1 MujocoSimWorld

文件：`src/hardware/mujoco_teleop_hardware.cpp`

功能：加载 MuJoCo XML、创建模型和窗口、管理渲染上下文。渲染线程和控制线程分离，避免渲染阻塞控制循环。

### 6.2 MujocoTeleopHardware

功能：按名字查找关节和 actuator，实现 `TeleopHardware` 接口。当前仿真采用偏运动学的跟踪方式：

```text
qpos = cmd.q
qvel = cmd.dq
ctrl = 0
```

### 6.3 关节映射

MuJoCo 关节按名字查找，不依赖裸数组顺序：

```text
openarm_left_joint1..7   -> left_joint1_ctrl..7_ctrl
openarm_right_joint1..7  -> right_joint1_ctrl..7_ctrl
```

## 7. 公共基础设施

### 7.1 common.hpp

文件：`src/common.hpp`

包含三个控制程序共用的基础设施：

- `keep_running` + `signal_handler()`：统一的 Ctrl+C 信号处理。
- `make_default_safety_config(total_joints)`：真机安全配置。
- `make_sim_safety_config(total_joints)`：仿真安全配置。

### 7.2 openarm_constants.hpp

文件：`src/openarm_constants.hpp`

全局常量定义，使用 `constexpr` 替代 `#define`：

- `NJOINTS = 8`、`NMOTORS = 8`：关节数和电机数。
- `ROLE_LEADER = 1`、`ROLE_FOLLOWER = 2`：角色标识。
- `position_limit_max/min`：位置限位（Leader 和 Follower 共用）。
- `velocity_limit`、`effort_limit`：速度和力矩限制。
- 通过 `#define` 别名保持向后兼容（如 `#define position_limit_max_F position_limit_max`）。

## 8. 对官方 C++ teleop 的主要改造点

- `Control` 增加了 `TeleopHardware` 后端入口，使同一个控制器可驱动 CAN 或 MuJoCo。
- `Control` 中接入 `SafetyFilter`，在 MITParam 发送前限制异常 reference。
- `RobotSystemState` 和 `JointState` 扩展为更适合 reference/response 同时管理的形式。
- `openarm_unilateral_control.cpp` 增加 replay/record 参数，用于实机主从遥操录制和 follower 回放。
- 新增 `openarm_mujoco_panel_control.cpp`，作为双窗口 MuJoCo C++ teleop 仿真入口。
- `CsvReplay` 增加目录批量回放、衔接轨迹生成、衔接文件管理功能。

官方主控制思想没有被改成 Python 控制，也没有改成 IK/笛卡尔控制。当前主链路仍然是：

```text
Control::unilateral_step()
  -> joint_to_motor
  -> MITParam
  -> hardware backend
```

## 9. 调试和验证工具

### 9.1 Bezier 关节空间轨迹生成

`script/generate_cubic_bezier_replay.py` 可根据 16 路 `q_start/q_goal` 生成三阶 Bezier replay CSV。

### 9.2 debug CSV

可通过 `--debug-csv` 或环境变量 `OPENARM_DEBUG_CSV` 启用。

记录字段：

```text
time,side,global_index,local_index,
q_target,dq_target,
q_ref,dq_ref,
q_actual,dq_actual,
tracking_error,
control_dt,input_update_dt
```

### 9.3 频率统计

`openarm_mujoco_panel_control.cpp` 每秒打印控制频率、backend 发送频率和渲染频率。正常仿真中控制和 backend send 应接近 1000 Hz/arm。

## 10. 文件功能速查

```text
control/openarm_mujoco_panel_control.cpp
```

双 MuJoCo 窗口仿真入口，负责 leader/follower、左右臂控制线程、录制、回放和 debug。

```text
control/openarm_unilateral_control.cpp
```

真机单边遥操作入口，支持 Live/Panel/Replay 三种参考源，支持录制和回放。

```text
control/openarm_bilateral_control.cpp
```

真机双边遥操作入口，leader 和 follower 之间力反馈双向控制。

```text
src/hardware/teleop_hardware.*
```

统一硬件抽象。`CanTeleopHardware` 对接真实 SocketCAN。

```text
src/hardware/mujoco_teleop_hardware.*
```

MuJoCo 硬件后端。负责模型加载、关节映射、状态读写和渲染。

```text
src/io/teleop_io.*
```

UDP 输入、CSV 录制、CSV 回放（含目录批量回放和衔接轨迹生成）。

```text
src/common.hpp
```

公共信号处理和安全配置函数。

```text
src/safety/safety_filter.*
```

安全限位层。

```text
src/openarm_constants.hpp
```

全局常量定义（constexpr）。

## 11. 推荐验证流程

构建：

```bash
cd /home/lan/openarm_demo2/openarm_inspace/openarm_inplace
source /home/lan/miniconda3/etc/profile.d/conda.sh
conda activate openarm_teleop1
cmake -S . -B build
cmake --build build -j2
```

单文件回放：

```bash
script/run_cpp_mujoco_replay.sh recordings/xxx.csv
```

目录批量回放：

```bash
script/run_cpp_mujoco_replay.sh recordings/
```

清理衔接文件后回放：

```bash
./build/mujoco_panel_control \
  --xml ../openarm_mujoco/v1/scene.xml \
  --urdf ../openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf \
  --replay recordings/ \
  --clean-transitions
```

判断链路正常的关键日志：

```text
[replay] loaded N frames from xxx.csv (seq=1)
[replay] total T frames from N file(s), duration Ds
[freq] control_per_arm_hz=...
[freq] backend_send_per_arm_hz=...
```
