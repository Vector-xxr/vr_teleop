# OpenArm VR 遥操（`vr_control`）

Meta Quest 双臂遥操与动作回放工程根目录。

---

## 环境配置（先做）

### Conda 环境

| 用途 | conda 环境 | Python | 说明 |
| --- | --- | --- | --- |
| VR 遥控 / Quest bridge | `openarm_quest` | — | `./quest/setup_venv.sh` 可建 |
| 任务编排 / 视觉抓取 / client | `openarm_teleop1` | **3.11** | `left_job_orchestrator`、`tcp_forward_grasp` |

```bash
# 激活示例
conda activate openarm_quest      # VR
conda activate openarm_teleop1   # 编排 / 抓取 / TCP client

# 或直接用绝对路径（推荐写进脚本）
PY_QUEST=/home/knowin/miniconda3/envs/openarm_quest/bin/python
PY_TELEOP=/home/knowin/miniconda3/envs/openarm_teleop1/bin/python
```

### CAN / 电机与夹爪（与真机一致）

默认接口与夹爪 MIT 在：

`openarm_teleop/openarm_inplace/script/openarm_vr_compat_env.sh`

| 项 | 默认 |
| --- | --- |
| 左 / 右 CAN | `follower_left` / `follower_right` |
| 夹爪行程 | open=`-0.98`，closed=`0.62` |
| 夹爪 MIT | Kp=`10`，Kd=`0.45` |
| 夹持保持 | 左关 / 右开；右手 lag≈`0.24`～`0.30` |

真机回放脚本会 `source` 该文件，**与 VR 遥操电机参数一致**。  
CAN 配置常需 sudo：

```bash
export OPENARM_SUDO_PASSWORD=knowin   # 按本机密码修改
```

### 常用路径

```text
工程根:     /home/knowin/Desktop/vr_control
录制/任务:  openarm_teleop/openarm_inplace/recordings/
VR 兼容环境: openarm_teleop/openarm_inplace/script/openarm_vr_compat_env.sh
```

日常命令汇总见 **[SOP.md](SOP.md)**。

---

## 文档索引

| 文档 | 内容 |
| --- | --- |
| **[SOP.md](SOP.md)** | 日常命令：环境 / VR / 录制 / 编排 / import_task / 抓取 |
| **[boot_autostart_left_job/](boot_autostart_left_job/README.md)** | 编排服务开机自启（systemd 配置包） |
| **[migration_build/](migration_build/README.md)** | 换机后统一编译/验证所有维护中的控制链路 |
| **[left_job_orchestrator/SOCKET_API.md](left_job_orchestrator/SOCKET_API.md)** | Socket：IP/端口/JSON 协议 |
| **[left_job_orchestrator/README.md](left_job_orchestrator/README.md)** | 整体动作回放服务 + 公共机 client |
| **[tcp_forward_grasp/README.md](tcp_forward_grasp/README.md)** | 视觉抓取单次跑通 |
| **[docs/teleop/](docs/teleop/README.md)** | 遥操流程 + 技术细节 |
| **[quest/SOP_QUEST_HUMAN_MIMIC.md](quest/SOP_QUEST_HUMAN_MIMIC.md)** | `human_mimic` 细则 |

---

## 四条常用链路（对照）

| 链路 | 入口 | 环境 |
| --- | --- | --- |
| VR 遥控 | `./quest/run_openarm_cartesian_bimanual.sh` | `openarm_quest` |
| 整体动作回放（job） | `left_job_orchestrator/./run_service.sh` + client | `openarm_teleop1` |
| 仅 import_task 轨迹 | `./run_import_task.sh` | bash + inplace 二进制 |
| 单次视觉抓取 | `tcp_forward_grasp/./run_once.sh` | `openarm_teleop1` |

---

## 目录结构（遥操相关）

```text
vr_control/
├── quest/                          # Bridge、WebXR、面板
├── openarm_teleop/openarm_inplace/ # C++ 控制 + recordings/
│   ├── script/openarm_vr_compat_env.sh
│   └── recordings/
│       ├── tasks/                  # steady / action CSV
│       ├── job1/                   # 编排任务（可含 visual_grasp*.txt）
│       ├── import_task/            # 纯轨迹回放目录
│       ├── sessions/               # VR 录制会话
│       └── hm_logs/                # 遥操诊断日志
├── left_job_orchestrator/          # 整体动作回放服务
├── tcp_forward_grasp/              # 视觉抓取
├── run_import_task.sh              # import_task 入口
├── docs/teleop/
└── SOP.md
```

---

## 一分钟上手（真机 VR）

```bash
cd /home/knowin/Desktop/vr_control

# 终端 1（可选：Bridge + FPV）
./quest/run_quest_camera_bridge.sh

# 终端 2（控制；必须带 human_mimic）
OPENARM_QUEST_EXTRA_ARGS="--teleop-mode human_mimic" \
  ./quest/run_openarm_cartesian_bimanual.sh
```

Quest 打开脚本打印的：`https://<局域网IP>:8443/client.html`

- **Grip 按住** → 跟手；松开 → 冻结  
- 夹爪：右 **A 闭 / B 开**，左 **X 闭 / Y 开**

---

## 运行链路（VR 简图）

```text
Quest WebXR → quest_bridge → UDP → quest_cartesian_control (human_mimic)
  → MIT-PD → SocketCAN (follower_left / follower_right)
```

详细时序与端口见 [docs/teleop/](docs/teleop/README.md)。

---

## 常用默认值

| 项 | 默认 |
| --- | --- |
| VR 模式 | `human_mimic`（需 EXTRA_ARGS） |
| HTTPS / WSS | `8443` / `8442` |
| 遥操 UDP 右 / 左 | `47911` / `47912` |
| 编排任务 TCP | `47925` |
| CAN | `follower_left` / `follower_right` |
| 夹爪 MIT | Kp=`10`, Kd=`0.45` |

---

## 停止与排障

- 停 VR：控制终端 `Ctrl+C` 或录制面板「退出」  
- `:8443` 被占：`ss -lntp | grep 8443`  
- 日志：`openarm_teleop/openarm_inplace/recordings/hm_logs/quest_<时间戳>/`  
- 编排与 `run_once` / `run_import_task` **不要同时抢同一套 CAN**
