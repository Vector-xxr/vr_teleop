# openarm_inplace

`openarm_inplace` 是当前实机遥操作、动作录制、动作回放、MuJoCo 验证和调试工具的主工程目录。它基于 OpenArm C++ 控制链路改造，当前推荐的真机主路径都走：

```text
leader / panel / replay CSV
  -> RobotSystemState reference
  -> Control::unilateral_step()
  -> SafetyFilter
  -> joint_to_motor
  -> MITParam
  -> CanTeleopHardware 或 MujocoTeleopHardware
```

16 关节 CSV 顺序统一为：

```text
left J1..J7, left gripper, right J1..J7, right gripper
```

## 快速开始

```bash
cd /home/lan/openarm_teleop/openarm_inspace/openarm_inplace
cmake -S . -B build
cmake --build build -j2
```

如果之前出现过 `CMakeCache.txt directory is different`，先清理 build：

```bash
rm -rf build
cmake -S . -B build
cmake --build build -j2
```

构建目标：

| 目标 | 功能 |
|---|---|
| `unilateral_control` | 单臂 leader-follower、单臂录制、单臂回放 |
| `dual_replay_control` | 双臂单进程同步回放、常驻回放 server |
| `follower_record_control` | 只读取 follower 端并录制 CSV |
| `mujoco_panel_control` | MuJoCo 双臂仿真、面板输入、CSV 回放 |
| `bilateral_control` | 旧双边控制入口 |
| `gravity_comp` | 重力补偿调试 |
| `comm_test` | CAN 通信测试 |
| `gripper_jog` | follower 夹爪往复 jog |
| `gripper_teleop_test` | 夹爪走完整 `Control::unilateral_step()` 的测试 |
| `gripper_only_teleop_test` | 仅夹爪电机 leader-follower 测试 |

默认 URDF：

```text
../openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf
```

默认 MuJoCo scene：

```text
../openarm_mujoco/v1/scene.xml
```

## 现场 CAN 命名

当前脚本默认使用稳定 SocketCAN 名称：

```text
右臂 leader   leader_right
右臂 follower follower_right
左臂 leader   leader_left
左臂 follower follower_left
```

大部分真机脚本默认会自动配置 CAN-FD：

```text
bitrate  = 1000000
dbitrate = 5000000
fd on
```

通用 CAN 环境变量：

| 变量 | 默认值 | 说明 |
|---|---:|---|
| `OPENARM_AUTO_CONFIG_CAN` | `1` | 脚本启动时自动 `ip link set ... type can ... fd on` |
| `OPENARM_SKIP_CAN_CONFIG` | `0` | 单臂脚本内部跳过 CAN 配置，双臂脚本拉起子进程时会用到 |
| `OPENARM_CAN_BITRATE` | `1000000` | CAN 仲裁段 bitrate |
| `OPENARM_CAN_DBITRATE` | `5000000` | CAN-FD data bitrate |
| `OPENARM_SUDO_PASSWORD` | 空 | 给录制/回放脚本非交互 sudo 用；用后脚本会 unset |
| `OPENARM_RIGHT_LEADER_CAN` | `leader_right` | 右臂 leader |
| `OPENARM_RIGHT_FOLLOWER_CAN` | `follower_right` | 右臂 follower |
| `OPENARM_LEFT_LEADER_CAN` | `leader_left` | 左臂 leader |
| `OPENARM_LEFT_FOLLOWER_CAN` | `follower_left` | 左臂 follower |
| `OPENARM_LEADER_CAN` | 根据 `OPENARM_TELEOP_ARM_SIDE` | 单臂 leader |
| `OPENARM_FOLLOWER_CAN` | 根据 `OPENARM_TELEOP_ARM_SIDE` | 单臂 follower |

动作录制和动作回放脚本默认会尝试自动配置 CAN。若 sudo 没有缓存且处于图形环境，脚本会弹出 `OpenArm sudo password` 密码框；无图形环境运行时，可提前设置 `OPENARM_SUDO_PASSWORD`，或设置 `OPENARM_AUTO_CONFIG_CAN=0` 后手动配置 CAN。

检查 CAN：

```bash
ip link show type can
ip -details -statistics link show leader_right
ip -details -statistics link show follower_right
ip -details -statistics link show leader_left
ip -details -statistics link show follower_left
```

## 真机动作录制

### 双臂 leader-follower 录制

```bash
script/run_cpp_real_teleop_record_both.sh test_action_01
```

输出：

```text
recordings/sessions/test_action_01/
├── motor_position_snapshots.txt
├── real_teleop_split/
│   ├── left/real_teleop_left.csv
│   └── right/real_teleop_right.csv
└── real_teleop_dual_merged.csv
```

启动后会弹出双臂录制面板。leader-follower 控制会先运行，点击面板 Start 后开始写 CSV，关闭面板或 `Ctrl+C` 后停止并自动合并左右 CSV。

双臂录制脚本参数和变量：

| 项 | 默认值 | 说明 |
|---|---:|---|
| 第 1 参数 | 自动 `real_YYYYMMDD_HHMMSS` | session 名 |
| `OPENARM_RECORD_NAME` | 空 | session 名，优先级低于第 1 参数 |
| `OPENARM_RECORD_SESSION_DIR` | `recordings/sessions/<name>` | session 输出目录 |
| `OPENARM_RIGHT_RECORD_PATH` | session 内 right split | 右臂 CSV |
| `OPENARM_LEFT_RECORD_PATH` | session 内 left split | 左臂 CSV |
| `OPENARM_MERGED_RECORD_PATH` | session 内 merged CSV | 左右合并 CSV |
| `OPENARM_POSITION_DUMP_PATH` | `motor_position_snapshots.txt` | 面板 dump 位姿输出 |
| `OPENARM_MERGED_RECORD_HZ` | `1000` | 合并 CSV 重采样频率 |
| `OPENARM_RIGHT_RECORD_START_PORT` | `47920` | 右臂录制 UDP 控制端口 |
| `OPENARM_LEFT_RECORD_START_PORT` | `47921` | 左臂录制 UDP 控制端口 |
| `OPENARM_RIGHT_REAL_TELEOP_LOG` | `/tmp/openarm_real_teleop_right.log` | 右臂日志 |
| `OPENARM_LEFT_REAL_TELEOP_LOG` | `/tmp/openarm_real_teleop_left.log` | 左臂日志 |
| `OPENARM_TELEOP_MONITOR_PANEL` | `1` | 是否启用 leader/follower 监控通道 |
| `OPENARM_TELEOP_MONITOR_AUTO_OPEN` | `0` | 是否自动打开监控面板 |
| `OPENARM_RIGHT_TELEOP_MONITOR_PORT` | `47950` | 右臂监控端口 |
| `OPENARM_LEFT_TELEOP_MONITOR_PORT` | `47951` | 左臂监控端口 |
| `OPENARM_RIGHT_TELEOP_HOME_POSE` | 空 | 右臂启动 home pose，覆盖全局值 |
| `OPENARM_LEFT_TELEOP_HOME_POSE` | 空 | 左臂启动 home pose，覆盖全局值 |

### 单臂 leader-follower 录制

右臂：

```bash
OPENARM_TELEOP_ARM_SIDE=right_arm \
OPENARM_LEADER_CAN=leader_right \
OPENARM_FOLLOWER_CAN=follower_right \
OPENARM_RECORD_PATH=recordings/right_demo.csv \
script/run_cpp_real_teleop_record.sh
```

左臂：

```bash
OPENARM_TELEOP_ARM_SIDE=left_arm \
OPENARM_LEADER_CAN=leader_left \
OPENARM_FOLLOWER_CAN=follower_left \
OPENARM_RECORD_PATH=recordings/left_demo.csv \
script/run_cpp_real_teleop_record.sh
```

单臂录制变量：

| 变量 | 默认值 | 说明 |
|---|---:|---|
| `OPENARM_TELEOP_URDF` | 默认 v1 URDF | 控制用 URDF |
| `OPENARM_TELEOP_ARM_SIDE` | `right_arm` | `right_arm` 或 `left_arm` |
| `OPENARM_RECORD_PATH` | `recordings/real_teleop_record.csv` | 输出 CSV |
| `OPENARM_POSITION_DUMP_PATH` | `recordings/motor_position_snapshots.txt` | dump 位姿文件 |
| `OPENARM_RECORD_START_PORT` | `47920` | Start/Stop/Dump UDP 端口 |
| `OPENARM_REAL_TELEOP_LOG` | `/tmp/openarm_real_teleop.log` | C++ 日志 |
| `OPENARM_TELEOP_LIVE_RAMP_DURATION` | `2.0` | follower 从当前位置平滑接入 leader 的时间 |
| `OPENARM_TELEOP_LIVE_VELOCITY_SCALE` | `0.0` | leader 速度前馈缩放 |
| `OPENARM_TELEOP_AUTO_HOME` | `1` | 是否启动时自动到 home pose |
| `OPENARM_TELEOP_HOME_POSE` | `~/Downloads/motor_p11.txt` 或 `../motor_p11.txt` | 启动目标位姿 |
| `OPENARM_TELEOP_LIVE_WAIT_FOR_START` | `0` | 是否点击 Start 后才接入 leader |
| `OPENARM_TELEOP_MONITOR_PORT` | 右 `47950`，左 `47951` | 单臂监控端口；`0` 关闭 |
| `OPENARM_NO_RECORD_PANEL` | `0` | `1` 时不弹录制面板，等待外部 UDP start |

### follower-only 录制

这个模式不使用 leader，只读取 follower 当前反馈并写 CSV，适合“直接拖 follower 录制，然后仍用 follower 回放”的流程。

脚本会 source `script/openarm_vr_compat_env.sh`，夹爪行程 / MIT（`OPENARM_GRIPPER_*`，默认 Kp=10）与 Quest 真机遥操一致，避免回放刚度不同。

双臂：

```bash
script/run_cpp_real_follower_record_both.sh test_action_01
```

单臂：

```bash
OPENARM_TELEOP_ARM_SIDE=right_arm \
OPENARM_FOLLOWER_CAN=follower_right \
script/run_cpp_real_follower_record.sh
```

变量：

| 变量 | 默认值 | 说明 |
|---|---:|---|
| `OPENARM_FOLLOWER_RECORD_HZ` | `1000` | 单臂 follower 采样频率 |
| `OPENARM_REAL_FOLLOWER_RECORD_LOG` | `/tmp/openarm_real_follower_record.log` | 单臂日志 |
| `OPENARM_RIGHT_REAL_FOLLOWER_RECORD_LOG` | `/tmp/openarm_real_follower_record_right.log` | 右臂日志 |
| `OPENARM_LEFT_REAL_FOLLOWER_RECORD_LOG` | `/tmp/openarm_real_follower_record_left.log` | 左臂日志 |

底层 `follower_record_control` 参数：

| 参数 | 说明 |
|---|---|
| `--arm-side right_arm|left_arm` | 记录哪一侧 |
| `--follower-can IFACE` | follower CAN 接口 |
| `--record CSV` | 输出 CSV |
| `--record-start-port PORT` | UDP 控制端口 |
| `--position-dump PATH` | dump 位姿文件 |
| `--hz HZ` | 采样频率 |
| `--keep-enabled-on-exit` | 退出时不 disable |

### 录制面板底层脚本

双臂录制脚本默认调用：

```bash
python script/record_start_panel_both.py \
  --right-port 47920 --left-port 47921 \
  --position-output recordings/sessions/test_action_01/motor_position_snapshots.txt
```

参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--host` | `127.0.0.1` | UDP host |
| `--right-port` | `47920` | 右臂 start/stop 控制端口 |
| `--left-port` | `47921` | 左臂 start/stop 控制端口 |
| `--right-log` | 空 | 右臂日志路径 |
| `--left-log` | 空 | 左臂日志路径 |
| `--position-output` | 空 | dump 位姿输出文件 |
| `--monitor-right-port` | `0` | 右臂监控端口；`0` 关闭 |
| `--monitor-left-port` | `0` | 左臂监控端口；`0` 关闭 |

单臂录制脚本默认调用：

```bash
python script/record_start_panel.py --port 47920 --arm-side right_arm
```

参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--host` | `127.0.0.1` | UDP host |
| `--port` | `47920` | start/stop 控制端口 |
| `--log` | 空 | 单臂日志路径 |
| `--position-output` | 空 | dump 位姿输出文件 |
| `--monitor-port` | `0` | 监控端口；`0` 关闭 |
| `--arm-side` | `right_arm` | 位姿 dump 使用的侧别名 |

## 真机动作回放

### 图形化回放面板

```bash
script/run_cpp_real_replay_panel.sh
```

面板会启动常驻 `dual_replay_control` server。可以选择：

- 单个 16 关节 merged CSV
- 标准 session 目录
- 整个 `recordings/sessions` 根目录
- 单臂 split CSV，面板会补齐另一臂静止轨迹
- 任意直接包含 16 关节 merged CSV 的文件夹

面板收到 server `ready` 后才允许发送动作命令；server 返回 `accepted` 并带回遥测端口后，才会打开对应的力矩面板，避免动作尚未接管时出现空监控窗口。关闭面板时会发送 `shutdown`，界面保持响应并等待双臂完成 Kd-only damping release，不默认强制杀掉回放进程。

面板相关变量：

| 变量 | 默认值 | 说明 |
|---|---:|---|
| `OPENARM_REPLAY_COMMAND_PORT` | `47970` | 面板发命令给 server |
| `OPENARM_REPLAY_STATUS_PORT` | `47971` | server 状态回传端口 |
| `OPENARM_REPLAY_PANEL_SPEED` | `1.0` | 面板速度框初始值 |
| `OPENARM_REPLAY_SHUTDOWN_WAIT` | 脚本内默认 | 重启常驻 server 时等待优雅退出时间；关闭面板时默认等待 damping release 完成 |
| `OPENARM_FACE_EMOJI_ENABLED` | `1` | 是否启用动作段表情联动 |
| `OPENARM_FACE_EMOJI_HOST` | `192.168.2.199` | face-emoji 地址 |
| `OPENARM_FACE_EMOJI_PORT` | `22345` | face-emoji 端口 |
| `OPENARM_FACE_EMOJI_TIMEOUT` | `5.0` | face-emoji 超时 |
| `OPENARM_TORQUE_PANEL_BASE_PORT` | `47980` | 面板为每次动作分配力矩遥测端口的起始值 |

底层面板也可以直接启动：

```bash
python script/real_replay_select_panel.py --project-dir /home/lan/openarm_teleop/openarm_inspace/openarm_inplace
```

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--project-dir PATH` | 当前脚本上级目录 | `openarm_inplace` 项目目录 |

动作 CSV 可在表头前写：

```text
# face_emoji=happy
time,q0,dq0,tau0,...
```

也支持：

```text
# face_emoji=scene:wave
# face_emoji={"type":"expression","intent":"happy","tts":"开始动作"}
```

### 命令行双臂回放

回放一个 session：

```bash
script/run_cpp_real_replay_both.sh recordings/sessions/test_action_01
```

回放最近一个 session：

```bash
script/run_cpp_real_replay_both.sh
```

回放整个 sessions 根目录：

```bash
script/run_cpp_real_replay_both.sh recordings/sessions
```

回放单个 merged CSV：

```bash
script/run_cpp_real_replay_both.sh recordings/sessions/test_action_01/real_teleop_dual_merged.csv
```

回放左右 split CSV：

```bash
script/run_cpp_real_replay_both.sh right.csv left.csv
```

双臂回放变量：

| 变量 | 默认值 | 说明 |
|---|---:|---|
| `OPENARM_RECORDINGS_SESSIONS_DIR` | `recordings/sessions` | 无参数时查找最近 session |
| `OPENARM_DUAL_REPLAY_SINGLE_PROCESS` | `1` | 优先使用 `dual_replay_control` 单进程同步回放 |
| `OPENARM_DUAL_REPLAY_INITIAL_TRANSITION_DURATION` | `3.0` | 当前/保持位姿到第一帧衔接时长；`0` 自适应 |
| `OPENARM_DUAL_REPLAY_TRANSITION_DURATION` | `5.0` | 动作文件之间衔接时长 |
| `OPENARM_DUAL_REPLAY_SPEED` | `1.0` | 回放速度倍率 |
| `OPENARM_REPLAY_SYNC_START_DELAY` | `1.5` | split fallback 双进程同步 start 前等待 |
| `OPENARM_REPLAY_INIT_HOLD_DURATION` | `0.5` | 启动时保持当前反馈姿态时间 |
| `OPENARM_REPLAY_STARTUP_GAIN_RAMP_DURATION` | `1.0` | 启动接管时 Kp/Kd ramp 时间 |
| `OPENARM_REPLAY_STARTUP_GAIN_RAMP_START_SCALE` | `0.1` | ramp 起始增益比例 |
| `OPENARM_REPLAY_AUTO_HOME` | `1` | 是否执行启动位姿流程 |
| `OPENARM_REPLAY_STARTUP_POSE` | `../motor_p11.txt` | 第一段启动张开位姿 |
| `OPENARM_REPLAY_HOME_POSE` | `../motor_p11.txt` | 动作准备位姿 |
| `OPENARM_TORQUE_PANEL` | `1` | 是否自动打开力矩/温度/错误码面板 |
| `OPENARM_TORQUE_TELEMETRY_PORT` | `47940` | 力矩遥测端口 |
| `OPENARM_TORQUE_TELEMETRY_OUTPUT` | replay 目录下 `torque_telemetry.csv` | 遥测 CSV |
| `OPENARM_TORQUE_RECOVERY_SPEED` | `0.25` | 安全恢复追帧速度 |
| `OPENARM_TORQUE_RECOVERY_MIN_DURATION` | `3.0` | 安全恢复最短时间 |
| `OPENARM_REPLAY_READY_TIMEOUT` | `60` | split fallback 等待单臂 ready 超时 |
| `OPENARM_RIGHT_REPLAY_PATH` | `recordings/real_teleop_split/right/real_teleop_right.csv` | 无 session 时右臂默认 CSV |
| `OPENARM_LEFT_REPLAY_PATH` | `recordings/real_teleop_split/left/real_teleop_left.csv` | 无 session 时左臂默认 CSV |
| `OPENARM_RIGHT_REPLAY_START_PORT` | `47930` | split fallback 右臂 start 端口 |
| `OPENARM_LEFT_REPLAY_START_PORT` | `47931` | split fallback 左臂 start 端口 |
| `OPENARM_RIGHT_REAL_REPLAY_LOG` | `/tmp/openarm_real_replay_right.log` | split fallback 右臂日志 |
| `OPENARM_LEFT_REAL_REPLAY_LOG` | `/tmp/openarm_real_replay_left.log` | split fallback 左臂日志 |

`dual_replay_control` 参数：

| 参数 | 说明 |
|---|---|
| `--urdf URDF` | 控制 URDF |
| `--replay CSV_OR_DIR` | 16 关节 merged CSV 或目录 |
| `--left-follower-can IFACE` | 左 follower CAN |
| `--right-follower-can IFACE` | 右 follower CAN |
| `--initial-transition-duration SEC` | 起点到第一帧衔接 |
| `--transition-duration SEC` | 动作之间衔接 |
| `--transition 0|1` | 是否生成衔接 |
| `--clean-transitions` | 清理衔接 CSV |
| `--start-delay SEC` | 初始化后等待 |
| `--speed SCALE` | 速度倍率 |
| `--init-hold-duration SEC` | 启动保持当前反馈 |
| `--startup-gain-ramp-duration SEC` | 启动增益 ramp 时间 |
| `--startup-gain-ramp-start-scale S` | 启动增益起始比例 |
| `--torque-recovery-speed RAD_PER_SEC` | 反馈异常恢复速度 |
| `--torque-recovery-min-duration SEC` | 恢复最短时间 |
| `--startup-pose PATH` | home 前的启动张开位姿 |
| `--home-pose PATH` | 动作准备位姿 |
| `--torque-telemetry-port PORT` | 遥测 UDP 端口 |
| `--command-port PORT` | 常驻 server 命令端口 |
| `--status-port PORT` | 常驻 server 状态端口 |
| `--no-auto-home` | 跳过启动位姿流程 |

常驻回放 server 也可以不经过图形面板直接启动：

```bash
script/run_cpp_real_replay_server.sh
```

它不接收 CSV 路径参数，而是等待 `real_replay_select_panel.py` 或其他 UDP client 向 `OPENARM_REPLAY_COMMAND_PORT` 发送播放/停止命令。可调变量与上表双臂回放一致，重点是 `OPENARM_REPLAY_COMMAND_PORT`、`OPENARM_REPLAY_STATUS_PORT`、`OPENARM_DUAL_REPLAY_SPEED`、`OPENARM_REPLAY_STARTUP_POSE`、`OPENARM_REPLAY_HOME_POSE`、`OPENARM_REPLAY_AUTO_HOME`、`OPENARM_REPLAY_INIT_HOLD_DURATION`、`OPENARM_REPLAY_STARTUP_GAIN_RAMP_DURATION`、`OPENARM_TORQUE_RECOVERY_SPEED`、`OPENARM_TORQUE_RECOVERY_MIN_DURATION`。

### 单臂回放

```bash
OPENARM_TELEOP_ARM_SIDE=right_arm \
OPENARM_LEADER_CAN=leader_right \
OPENARM_FOLLOWER_CAN=follower_right \
script/run_cpp_real_replay.sh recordings/sessions/test_action_01/real_teleop_split/right/real_teleop_right.csv
```

`run_cpp_real_replay.sh` 第 1 参数为 CSV 或目录，也可用 `OPENARM_REPLAY_PATH`。其余参数会透传给 `unilateral_control`。

`unilateral_control` 参数：

| 参数 | 说明 |
|---|---|
| `--reference-source live|panel|replay` | reference 来源，默认 live |
| `--panel-port PORT` | UDP 面板输入端口 |
| `--record PATH` | 录制 reference CSV |
| `--record-start-port PORT` | 等 UDP start 后录制 |
| `--position-dump PATH` | UDP dump 位姿输出 |
| `--replay PATH` | CSV/目录回放并设为 replay |
| `--replay-start-port PORT` | 等 UDP start 后开始回放 |
| `--transition 0|1` | 是否生成衔接 |
| `--transition-duration SEC` | 动作间衔接 |
| `--initial-transition-duration SEC` | 起点到第一帧衔接 |
| `--clean-transitions` | 删除衔接文件 |
| `--live-ramp-duration SEC` | live 接入平滑时间 |
| `--live-velocity-scale SCALE` | live 速度前馈比例 |
| `--live-wait-for-start 0|1` | live 是否等待 start |
| `--monitor-port PORT` | 监控 UDP 输出 |
| `--home-pose PATH` | 启动 home pose |
| `--no-auto-home` | 跳过启动回 home |

## MuJoCo 仿真与面板

启动 MuJoCo 加 16 关节滑块面板：

```bash
script/run_cpp_mujoco_with_panel.sh
```

只启动 MuJoCo 控制核心：

```bash
script/run_cpp_mujoco_panel.sh
```

用 MuJoCo 直接回放 CSV 或目录：

```bash
script/run_cpp_mujoco_replay.sh recordings/sessions/test_action_01
```

`run_cpp_mujoco_replay.sh` 第 1 参数为 CSV 或目录，也可用 `OPENARM_REPLAY_PATH`；剩余参数会继续透传给 `mujoco_panel_control`，例如 `--loop`、`--repeat 3`、`--transition 1`。

MuJoCo 脚本变量：

| 变量 | 默认值 | 说明 |
|---|---:|---|
| `OPENARM_MUJOCO_XML` | `../openarm_mujoco/v1/scene.xml` | MuJoCo XML |
| `OPENARM_TELEOP_URDF` | 默认 v1 URDF | 控制 URDF |
| `OPENARM_TELEOP_SIDE` | `right` | 单臂模式侧别 |
| `OPENARM_PANEL_PORT` | `47910` | UDP 面板端口 |
| `OPENARM_RECORD_PATH` | `recordings/mujoco_panel.csv` | MuJoCo 录制输出 |
| `OPENARM_DEBUG_CSV` | 空 | 调试 CSV |
| `OPENARM_MUJOCO_LOG` | `/tmp/openarm_cpp_mujoco.log` | with-panel 模式 C++ 日志 |
| `OPENARM_TELEOP_ENV` | `openarm_teleop1` | `run_cpp_joint_panel.sh` 激活的 conda 环境 |

`mujoco_panel_control` 参数：

| 参数 | 说明 |
|---|---|
| `--xml XML` | MuJoCo scene |
| `--urdf URDF` | 控制 URDF |
| `--side SIDE` | 单臂侧别 |
| `--panel-port PORT` | UDP 输入端口 |
| `--record CSV` | 录制输出 |
| `--replay PATH` | CSV/目录回放 |
| `--transition 0|1` | 是否生成衔接 |
| `--clean-transitions` | 清理衔接文件 |
| `--debug-csv CSV` | 调试输出 |
| `--loop` | 无限循环 |
| `--repeat N` | 重复次数 |

滑块面板：

```bash
script/run_cpp_joint_panel.sh --port 47910 --arms bimanual --hz 50
```

底层 Python 入口等价于：

```bash
python script/cpp_joint_panel.py --port 47910 --arms bimanual --hz 50
```

参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--host` | `127.0.0.1` | UDP host |
| `--port` | `47910` | UDP port |
| `--hz` | `50` | 发送频率 |
| `--arms single|bimanual` | `single` | 控制单臂或双臂 |

## CSV 工具

### 发送 CSV 到 MuJoCo leader

```bash
python script/send_csv_actions.py recordings/ --port 47910 --speed 1.0
```

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `path` | 必填 | CSV 文件或目录 |
| `--host` | `127.0.0.1` | UDP host |
| `--port` | `47910` | UDP port |
| `--speed` | `1.0` | 播放速度 |
| `--positions-only` | false | 只发送 16 个 q，不发 dq |
| `--loop` | false | 循环播放 |
| `--repeat` | `1` | 重复次数 |
| `--print-every` | `2.0` | 进度打印间隔 |

### 发送平滑测试轨迹

```bash
python script/send_smooth_joint_motion.py \
  --side both --profile constant --duration 30 --cycles 2 \
  --amplitude 0.75 --hz 500 --velocity-log recordings/smooth_velocity_command.csv
```

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--host` | `127.0.0.1` | UDP host |
| `--port` | `47910` | UDP port |
| `--hz` | `500` | 发送频率 |
| `--duration` | `30` | 总时长 |
| `--cycles` | `2` | 周期数 |
| `--amplitude` | `0.75` | 运动幅值 |
| `--side left|right|both` | `both` | 运动侧 |
| `--profile constant|sine` | `constant` | 三角匀速或正弦 |
| `--gripper` | `0.0` | 夹爪位置 |
| `--positions-only` | false | 只发 q |
| `--print-every` | `1.0` | 打印间隔 |
| `--velocity-log` | 空 | 记录命令 q/dq |

### 生成 Bezier CSV

```bash
python script/generate_cubic_bezier_replay.py \
  --q-start "q0,...,q15" \
  --q-goal "q0,...,q15" \
  --duration 5 --hz 200 \
  --output recordings/bezier.csv
```

也可以从 CSV 取首末帧：

```bash
python script/generate_cubic_bezier_replay.py \
  --start-csv a.csv --start-row 0 \
  --goal-csv b.csv --goal-row -1 \
  --output recordings/transition.csv
```

### 平滑 CSV

```bash
python script/smooth_replay_csv.py input.csv -o output.csv
```

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `input` | 内置示例路径 | 输入 CSV，不修改原文件 |
| `-o, --output` | `INPUT_STEM_smooth.csv` | 输出 CSV |
| `--cutoff-hz` | `6.0` | 位置低通截止频率 |
| `--velocity-cutoff-hz` | `10.0` | 速度低通；`<=0` 关闭 |
| `--no-preserve-endpoints` | false | 不强制保持首末位置 |
| `--no-zero-end-velocity` | false | 不强制首末速度为 0 |
| `--max-velocity` | `0` | 速度限幅；`<=0` 关闭 |
| `--max-acceleration` | `0` | 加速度限幅；`<=0` 关闭 |

### 合并左右臂录制

```bash
python script/merge_dual_real_recordings.py \
  --left left.csv --right right.csv --output merged.csv --hz 1000
```

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--left` | 必填 | 左臂 CSV |
| `--right` | 必填 | 右臂 CSV |
| `--output` | 必填 | 输出 merged CSV |
| `--hz` | 自动推断 | 输出频率 |
| `--crop min|max` | `min` | `min` 只保留重叠时长；`max` 用最近帧补齐 |

### 准备目录回放顺序

```bash
python script/prepare_dual_replay_directory.py recordings/sessions/test_action_01
```

参数：

| 参数 | 说明 |
|---|---|
| `root` | session 或 sessions 根目录 |
| `--output-root PATH` | 自定义输出整理目录 |

## 监控面板

### 录制监控

```bash
python script/teleop_monitor_panel.py --right-port 47950 --left-port 47951
```

显示 leader/follower 反馈、夹爪 contact-hold/slip-boost 状态、断流 stale 状态。

| 参数 | 默认值 |
|---|---:|
| `--host` | `127.0.0.1` |
| `--right-port` | `47950` |
| `--left-port` | `47951` |
| `--stale` | `0.5` |

### 力矩/温度/错误码面板

```bash
python script/torque_telemetry_panel.py --port 47940 --output torque.csv
```

显示 16 电机：

- 反馈力矩曲线
- `MOS` 温度
- `rotor` 温度
- `reserved_alert_mask` 兼容保留列，当前固定为 `0`
- 电机故障码

该面板只消费动作回放进程发出的力矩遥测 UDP 数据，不再依赖心跳/断联标记。若状态栏 `samples` 增长但曲线区域为空，请确认正在运行的是更新后的 `torque_telemetry_panel.py`。

故障码含义：

| code | 含义 |
|---:|---|
| 8 | 过压 |
| 9 | 欠压 |
| 10 | 过流 |
| 11 | MOS 过温 |
| 12 | 电机线圈/rotor 过温 |
| 13 | 通信丢失 |
| 14 | 过载 |

参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--host` | `127.0.0.1` | UDP host |
| `--port` | `47940` | UDP port |
| `--window` | `10.0` | 实时显示窗口秒数 |
| `--torque-limit` | `20.0` | 初始力矩坐标尺度 |
| `--output` | 空 | 保存 CSV |
| `--title` | 空 | 窗口标题 |

## 夹爪调试

### follower 夹爪 jog

```bash
script/jog_follower_gripper.sh
```

变量：

| 变量 | 默认值 | 说明 |
|---|---:|---|
| `OPENARM_FOLLOWER_CAN` | `follower_right` | follower CAN |
| `OPENARM_GRIPPER_JOG_DELTA` | `0.08` | 往复位移 |
| `OPENARM_GRIPPER_JOG_STEP_DURATION` | `0.8` | 每段运动时间 |
| `OPENARM_GRIPPER_JOG_SETTLE_DURATION` | `0.3` | 停留时间 |
| `OPENARM_GRIPPER_JOG_KP` | `8.0` | Kp |
| `OPENARM_GRIPPER_JOG_KD` | `0.3` | Kd |

底层参数同名：`--can`、`--delta`、`--step-duration`、`--settle-duration`、`--kp`、`--kd`。

### 完整控制链路夹爪测试

```bash
script/test_gripper_teleop_only.sh right
```

变量：

| 变量 | 默认值 |
|---|---:|
| `OPENARM_GRIPPER_TEST_HZ` | `1000` |
| `OPENARM_GRIPPER_TEST_PREZERO_DURATION` | `2.0` |
| `OPENARM_GRIPPER_TEST_PREZERO_TARGET` | `0.0` |

### gripper_only_teleop_test

脚本封装：

```bash
script/run_cpp_real_gripper_teleop_test.sh --side both --hz 200 --kp 8.0 --kd 0.3
```

这个脚本只初始化/使能夹爪电机，不会初始化、保持或控制大臂关节。它会同时启动左右两侧时把右侧日志写到 `OPENARM_RIGHT_GRIPPER_TELEOP_LOG`，左侧日志写到 `OPENARM_LEFT_GRIPPER_TELEOP_LOG`。

脚本参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--side left|right|both` | `both` | 启动哪侧夹爪 |
| `--hz HZ` | `200` | 控制频率 |
| `--kp K` | `8.0` | follower 夹爪 Kp |
| `--kd D` | `0.3` | follower 夹爪 Kd |
| `--offset RAD` | `0.0` | leader 到 follower 的位置偏置 |
| `--send-id ID` | `0x08` | 夹爪 send CAN ID |
| `--recv-id ID` | `0x18` | 夹爪 recv CAN ID |

脚本变量：

| 变量 | 默认值 | 说明 |
|---|---:|---|
| `OPENARM_GRIPPER_TELEOP_TEST_SIDE` | `both` | 等价于 `--side` |
| `OPENARM_RIGHT_GRIPPER_TELEOP_LOG` | `/tmp/openarm_gripper_teleop_right.log` | 右侧日志 |
| `OPENARM_LEFT_GRIPPER_TELEOP_LOG` | `/tmp/openarm_gripper_teleop_left.log` | 左侧日志 |
| `OPENARM_GRIPPER_TELEOP_HZ` | `200` | 等价于 `--hz` |
| `OPENARM_GRIPPER_TELEOP_KP` | `8.0` | 等价于 `--kp` |
| `OPENARM_GRIPPER_TELEOP_KD` | `0.3` | 等价于 `--kd` |
| `OPENARM_GRIPPER_TELEOP_OFFSET` | `0.0` | 等价于 `--offset` |
| `OPENARM_GRIPPER_SEND_CAN_ID` | `0x08` | 等价于 `--send-id` |
| `OPENARM_GRIPPER_RECV_CAN_ID` | `0x18` | 等价于 `--recv-id` |

底层 C++ 参数：

| 参数 | 说明 |
|---|---|
| `--leader-can IFACE` | leader CAN |
| `--follower-can IFACE` | follower CAN |
| `--send-id HEX_OR_DEC` | 夹爪 send id |
| `--recv-id HEX_OR_DEC` | 夹爪 recv id |
| `--hz HZ` | 控制频率 |
| `--kp K` | Kp |
| `--kd D` | Kd |
| `--offset RAD` | leader 到 follower 映射偏置 |
| `--min-position RAD` | 最小位置 |
| `--max-position RAD` | 最大位置 |

### 生成夹爪测试 CSV 并走真实回放

```bash
script/run_cpp_real_gripper_test.sh --side both --open 0.0 --close 0.035 --cycles 3
```

该脚本先根据保持位姿生成一个 `real_teleop_dual_merged.csv`，只让选中的夹爪做五次多项式平滑开合，其他 15 个关节保持 `--hold-pose` 里的 follower 位置。默认生成后会调用真实双臂回放链路；加 `--no-run` 时只生成 CSV。

参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--side left|right|both` | `both` | 测试哪侧夹爪 |
| `--open VALUE` | `0.0` | 开合起点/打开位置 |
| `--close VALUE` | `0.035` | 闭合位置 |
| `--cycles N` | `3` | open-close-open 周期数 |
| `--cycle-duration SEC` | `3.0` | 每个周期秒数 |
| `--hz HZ` | `200` | 生成 CSV 频率 |
| `--effort VALUE` | `0.0` | 夹爪 tau/feedforward 列 |
| `--hold-pose PATH` | `recordings/sessions/test_zhengbei2/initial_p1.txt` | 非测试关节保持位姿 |
| `--output-dir DIR` | `/tmp/openarm_gripper_test.*` | 生成 CSV 目录 |
| `--no-run` | false | 只生成 CSV，不启动真实回放 |

同名环境变量均支持，包括 `OPENARM_GRIPPER_TEST_*` 前缀下的 `OPENARM_GRIPPER_TEST_SIDE`、`OPENARM_GRIPPER_TEST_OPEN`、`OPENARM_GRIPPER_TEST_CLOSE`、`OPENARM_GRIPPER_TEST_CYCLES`、`OPENARM_GRIPPER_TEST_CYCLE_DURATION`、`OPENARM_GRIPPER_TEST_HZ`、`OPENARM_GRIPPER_TEST_EFFORT`、`OPENARM_GRIPPER_TEST_HOLD_POSE`、`OPENARM_GRIPPER_TEST_OUTPUT_DIR`。

## 旧 ROS/xacro 启动脚本

这些脚本依赖 `~/openarm_ros2_ws` 和 xacro 生成 URDF，当前不是推荐主路径：

```bash
script/launch_unilateral.sh right_arm [leader_can] [follower_can]
script/launch_bilateral.sh right_arm [leader_can] [follower_can]
script/launch_grav_comp.sh right_arm [can_if] [arm_type]
```

## 控制与安全环境变量

### 启动位姿

| 变量 | 默认值 | 说明 |
|---|---:|---|
| `OPENARM_ADJUST_POSITION_SPEED` | `0.25` | `AdjustPosition()` 最大平滑速度 |
| `OPENARM_ADJUST_POSITION_MIN_DURATION` | `4.0` | `AdjustPosition()` 最短时间 |
| `OPENARM_INIT_DISABLE_BEFORE_ENABLE` | `0` | 初始化时先 disable 再 enable |
| `OPENARM_LEADER_FRICTION_SCALE` | `0.3` | leader 侧摩擦补偿缩放，范围 `0.0` 到 `2.0` |

录制启动位姿由 `OPENARM_TELEOP_HOME_POSE` 控制。回放启动位姿分两段：

```text
当前反馈 -> OPENARM_REPLAY_STARTUP_POSE -> OPENARM_REPLAY_HOME_POSE -> replay 第一帧
```

### 夹爪接触保持

默认不启用 contact-hold；只有设置 `OPENARM_GRIPPER_CONTACT_HOLD=1` 才会生效。它不是电机原生 `POS_FORCE`，而是在 MIT 位置控制上锁定接触位置并可选 slip-boost。

| 变量 | 默认值 |
|---|---:|
| `OPENARM_GRIPPER_CONTACT_HOLD` | `0` |
| `OPENARM_GRIPPER_CONTACT_EFFORT` | `0.30` |
| `OPENARM_GRIPPER_STALL_VELOCITY` | `0.08` |
| `OPENARM_GRIPPER_CONTACT_POSITION_ERROR` | `0.01` |
| `OPENARM_GRIPPER_CONTACT_HOLD_TIME` | `0.20` |
| `OPENARM_GRIPPER_RELEASE_DELTA` | `0.06` |
| `OPENARM_GRIPPER_RELEASE_HOLD_TIME` | `0.08` |
| `OPENARM_GRIPPER_HOLD_KP_SCALE` | `0.8` |
| `OPENARM_GRIPPER_SLIP_BOOST` | `1` |
| `OPENARM_GRIPPER_SLIP_BOOST_KP_SCALE` | `1.15` |
| `OPENARM_GRIPPER_SLIP_BOOST_POSITION_DELTA` | `0.012` |
| `OPENARM_GRIPPER_SLIP_BOOST_VELOCITY` | `0.04` |
| `OPENARM_GRIPPER_SLIP_BOOST_HOLD_TIME` | `0.25` |
| `OPENARM_GRIPPER_CLOSE_DIRECTION` | `0.0` |

### 退出行为

当前双臂动作回放结束或安全停止时，不再自动回零；会发送 Kd-only damping release：

```text
Kp = 0
Kd = follower.yaml 中对应 Kd
q  = 当前反馈位置
dq = 0
tau = 0
```

电机保持 enabled，让机械臂在阻尼下慢慢释放。录制/遥操作链路退出时仍会 disable leader/follower。

## CSV 格式

推荐格式：

```text
time,q0,dq0,tau0,q1,dq1,tau1,...,q15,dq15,tau15
```

旧格式仍兼容：

```text
time,q0,dq0,q1,dq1,...,q15,dq15
```

注释行支持动作元数据，例如：

```text
# face_emoji=happy
```

## 常见命令

编译：

```bash
cmake -S . -B build
cmake --build build -j2
```

录制双臂：

```bash
script/run_cpp_real_teleop_record_both.sh test_action_01
```

follower-only 录制：

```bash
script/run_cpp_real_follower_record_both.sh test_action_01
```

打开回放面板：

```bash
script/run_cpp_real_replay_panel.sh
```

命令行回放：

```bash
script/run_cpp_real_replay_both.sh recordings/sessions/test_action_01
```

关闭回放力矩面板：

```bash
OPENARM_TORQUE_PANEL=0 script/run_cpp_real_replay_both.sh recordings/sessions/test_action_01
```

降低回放速度：

```bash
OPENARM_DUAL_REPLAY_SPEED=0.5 script/run_cpp_real_replay_both.sh recordings/sessions/test_action_01
```
