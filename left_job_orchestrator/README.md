# Left Job Orchestrator

常驻左臂任务编排：steady 一次后空闲，**TCP** `:47925` 触发 `job1` / `job2`。  
协议与对接说明见 **[SOCKET_API.md](SOCKET_API.md)**（明文 TCP + Token；TLS/`sig` 暂关）。

---

## 环境配置（先做）

### Conda / Python

本模块需要 **conda `openarm_teleop1`（Python 3.11）**。`./run_service.sh` 会优先用该环境的解释器。

```bash
conda activate openarm_teleop1

# 或显式指定（公共机 client 推荐）
export PY=/home/knowin/miniconda3/envs/openarm_teleop1/bin/python
# 可选覆盖服务用的解释器：
# export OPENARM_PYTHON=$PY
```

### CAN / 电机 / sudo

- CAN 默认：`follower_left` / `follower_right`
- 电机与夹爪：**与 VR 遥操相同**  
  （`run_cpp_real_replay_server.sh` → `source openarm_vr_compat_env.sh`）
- 真机启动前：

```bash
export OPENARM_SUDO_PASSWORD=你的sudo密码
```

### 可选环境变量

| 变量 | 含义 | 默认 |
| --- | --- | --- |
| `OPENARM_SKEWER_JOBS_ROOT` | job1/job2 所在根目录 | `…/recordings` |
| `OPENARM_SKEWER_JOB_LISTEN_PORT` | 任务 **TCP** 端口 | `47925` |
| `OPENARM_LEFT_JOB_API_TOKEN` | JSON API 共享密钥（token） | 默认 `openarm-job-api` |
| `OPENARM_LEFT_JOB_REQUIRE_AUTH` | `1` 强制 JSON+token；`0` 关闭 | 默认开 |
| `OPENARM_LEFT_JOB_SUBSCRIBE_HEARTBEAT_S` | 订阅推送心跳间隔（秒） | `2` |
| `OPENARM_LEFT_JOB_PREVIEW` | `1` 时开相机预览 | 关 |
| `OPENARM_SUDO_PASSWORD` | CAN 配置 sudo | — |
| `OPENARM_ROS_SETUP` | ROS setup（systemd 必须显式加载） | `/opt/ros/${ROS_DISTRO:-humble}/setup.bash` |

### 工程路径

```bash
cd /home/knowin/Desktop/vr_control
# 服务：left_job_orchestrator/
# 任务：openarm_teleop/openarm_inplace/recordings/jobN/
```

勿与 VR / `tcp_forward_grasp/run_once.sh` / `run_import_task.sh` **同时**抢同一套 CAN。

---

## 流程

```text
启动服务
  → 起 dual_replay_control（VR 同款电机配置）
  → 播 recordings/tasks/steady_action_on_table.csv（一次）
  → 空闲，accept TCP :47925
  → 辅助线程：向已 subscribe 的 TCP 连接推送 event|status...

收到 1 / job1  →  recordings/job1/
  → 扫描 actionN.csv / visual_graspN.txt
  → 按文件名末尾数字执行，例如:
    action1.csv → visual_grasp2.txt → action2.csv → action3.csv
```

执行中再发任务数字会回 `busy|job=...`（`status` / `subscribe` / `abort` / `shutdown` 仍可用）。

---

## 目录约定

```text
openarm_teleop/openarm_inplace/recordings/
  tasks/steady_action_on_table.csv
  job1/
    action1.csv              # 必需；任务至少有这一段
    visual_grasp2.txt        # 可选：第 2 步执行视觉抓取
    action2.csv              # 可选：视觉抓取后执行第 2 段轨迹
    action3.csv              # 可选：第 3 段轨迹
    job.json                 # 可选
  job2/...

openarm_teleop/openarm_inplace/left_grasp_exec_bundle/begin.csv
tcp_forward_grasp/config/gripper_gate_left.json
```

规则：

- 动作数量不限，只有 `action1.csv` 也可以。
- `actionN.csv` 与 `visual_graspN.txt` 按末尾数字 `N` 排序，数字可不连续。
- 同编号时先执行 `visual_graspN.txt`，再执行 `actionN.csv`。
- 同类型的同一编号不能重复。
- 视觉抓取前必须是动作；如果后面还有步骤，下一步也必须是动作。
- `visual_grasp.txt` 兼容旧任务，固定放在 `action1` 和 `action2` 之间。
- `visual_grispN.txt` 拼写也兼容，但推荐统一使用 `visual_graspN.txt`。

`job.json` 可覆盖 begin 位姿等（键名兼容旧 observe）：

```json
{
  "begin_csv": "/abs/path/to/begin.csv",
  "post_grasp_retreat_mm": 140
}
```

---

## 启动（机器人 / 服务端）

```bash
cd /home/knowin/Desktop/vr_control/left_job_orchestrator
export OPENARM_SUDO_PASSWORD=你的sudo密码
./run_service.sh
```

systemd 启动不会继承交互 shell 的 ROS 环境。部署时在 `env.systemd` 确认
`OPENARM_ROS_SETUP`，并使用 `boot_autostart_left_job/install.sh --enable`
执行无硬件预检；详见部署包 [README](../boot_autostart_left_job/README.md)。

可选开相机预览：

```bash
OPENARM_LEFT_JOB_PREVIEW=1 ./run_service.sh
```

---

## 公共机（client）

```bash
cd /home/knowin/Desktop/vr_control
# 将 <robot-ip> 换成机器人局域网 IP；本机联调可用 127.0.0.1
PY=/home/knowin/miniconda3/envs/openarm_teleop1/bin/python

$PY -m left_job_orchestrator.client --host <robot-ip> status
$PY -m left_job_orchestrator.client --host <robot-ip> 1
$PY -m left_job_orchestrator.client --host <robot-ip> 2
$PY -m left_job_orchestrator.client --host <robot-ip> abort
$PY -m left_job_orchestrator.client --host <robot-ip> shutdown

# 订阅状态推送（TCP 长连接；心跳 + 状态变化）
$PY -m left_job_orchestrator.client --host <robot-ip> subscribe
```

裸 TCP（任意语言）：连上后发一行命令 + `\n`，读一行应答。

**推荐（正式）：JSON 行协议**（见 `SOCKET_API.md`）

```text
{"v":1,"cmd":"status","id":"a1","ts":1710000000.123,"token":"openarm-job-api"}
{"v":1,"cmd":"run","job":"1","id":"a2","ts":...,"token":"openarm-job-api"}
```

鉴权：请求带相同 `token` 即可（`sig` 暂不要求）。  
传输默认 **明文 TCP**；需要加密时设 `OPENARM_LEFT_JOB_TLS=1`。

**测试程序：**

```bash
$PY -m left_job_orchestrator.test_socket --host <robot-ip> status
$PY -m left_job_orchestrator.test_socket --host <robot-ip> run --job 1
$PY -m left_job_orchestrator.test_socket --host <robot-ip> interactive
```

兼容旧明文：`status\n` / `1\n`（仅当未开启强制鉴权时）。

```text
status\n          → ok|busy=0|... 或 JSON（若对端先发过 JSON）
1\n               → accepted|job=1|...  或 busy|job=1
subscribe\n       → ok|subscribed|...  随后同连接推送
```

---

## 端口

| 用途 | 默认 | 说明 |
|------|------|------|
| 任务命令 / 订阅 | **TCP 47925** | 公共机 client ↔ 本服务 |
| Replay 控制/状态 | dual_replay 默认（约 UDP 47970/47971） | 勿与其它真机进程抢 CAN |

---

## 模块

| 文件 | 作用 |
|------|------|
| `service.py` | 常驻服务、TCP、job 线程、subscribe 推送线程 |
| `protocol.py` | JSON 行协议 + token 鉴权 |
| `test_socket.py` | 公共机 / 本机 socket 联调（status/run/interactive） |
| `client.py` | 公共机 TCP 客户端（含 subscribe；兼容旧明文） |
| `pipeline.py` | 按编号编排任意数量的动作与视觉抓取步骤 |
| `replay.py` | 起 server + steady / shutdown |
| `run_service.sh` | 选 conda 解释器并启动 service |

日常总览见仓库根目录 [SOP.md](../SOP.md) / [README.md](../README.md)。
