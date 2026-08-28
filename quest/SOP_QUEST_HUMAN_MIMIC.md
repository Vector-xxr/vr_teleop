# Quest 遥操 SOP：`human_mimic`（逆解 + 虚拟骨架）

适用工程目录：`vr_control`（路径均相对工程根）。

本模式：

- **VR 手柄**：末端位姿 + grip clutch
- **VirtualArmSkeleton**：肩肘构型约束
- **整臂 IK**（默认 diff-QP）：腕 6D + 肘 XYZ → `q_target`（关节**位置**）→ `JointReferenceGenerator` → MIT-PD
- **Soft-reach 默认开**：中段弯肘；探底才近直（见 §3）
- **坐标系**：位置 VR 世界→基座固定轴；姿态重力锁定 + clutch 相对重锚
- **细动**：连续限速 φ + q slew（默认关 clutch 肘硬锁）；路径连续优先于硬够到

工程文档入口：根目录 [`README.md`](../README.md)、运行链路 [`docs/VR_TELEOP_PIPELINE.md`](../docs/VR_TELEOP_PIPELINE.md)、技术说明 [`docs/VR_TELEOP_TECH.md`](../docs/VR_TELEOP_TECH.md)。  
日常上手与 **完整环境变量表** 见根目录 [`SOP.md`](../SOP.md)。

| 模式 | 日常？ | 要点 |
| --- | --- | --- |
| **`human_mimic`** | **是** | soft-reach + 肘骨架（本文） |
| `pika_style` | 试验 | 6D、无 soft-reach/肘；易崩直/拧肘 |
| 其它 | 否 | `cartesian_precision` / `skeleton_proximal` 见 §4 |

---

## 0. 启动前检查

| 项 | 说明 |
| --- | --- |
| 工程根 | `cd openarm_teleop1` |
| Conda | `openarm_quest`（`OPENARM_QUEST_CONDA_ENV` 可改） |
| 控制程序 | 脚本自动编译 `openarm_teleop/openarm_inplace/build/quest_cartesian_control` |
| Quest | 脚本打印 `https://<LAN-IP>:8443/client.html` |
| Grip | **按住** 才跟手 |

默认 UDP：右臂 `47911`，左臂 `47912`。停止：`Ctrl+C` 或录制面板 **退出**。

---

## 1. 真机双臂

### 1.1 安全

1. 急停可用、工作区无人
2. CAN：左 `can1`、右 `can0`
3. **启动动作（默认）**：走到 `recordings/tasks/steady_action_on_table.csv`（过渡约 5 s，hold 约 0.6 s）
   - 改默认：编辑 `quest/run_openarm_cartesian_bimanual.sh` 顶部 `OPENARM_QUEST_STARTUP_*` 默认值
   - 跳过：`OPENARM_QUEST_STARTUP_MOVE=0`
   - 改回 TABLE_READY：`OPENARM_QUEST_STARTUP_FILE=`
   - 换文件：`OPENARM_QUEST_STARTUP_FILE=path/to/ready.csv`

### 1.2 两个终端（推荐）

**终端 1 — Quest + FPV 相机（可选）**

```bash
cd openarm_teleop1
./quest/run_quest_camera_bridge.sh
```

**终端 2 — 控制 + 录制 / 点动面板**

```bash
cd openarm_teleop1

OPENARM_QUEST_EXTRA_ARGS="--teleop-mode human_mimic" \
  ./quest/run_openarm_cartesian_bimanual.sh
```

- 若终端 1 已占用 `:8443`，终端 2 **复用** bridge，不再重复启动
- 默认弹出 **动作录制** + **TCP 微调** 两个 Tk 面板（`OPENARM_QUEST_UI=0` 关闭）
- 录制数据：`openarm_teleop/openarm_inplace/recordings/sessions/<会话名>/<段号>/`
- 回放：`./script/run_cpp_real_replay_both.sh <段目录>` 或 `./script/run_cpp_real_replay_panel.sh`

不需要 FPV 时只开终端 2 即可（脚本自带无相机 bridge）。

### 1.3 可选 HybrIK 肘 φ（第三终端）

```bash
cd HybrIK
./run_swivel_bridge.sh
```

控制命令加 `--elbow-hint hybrik`：

```bash
OPENARM_QUEST_EXTRA_ARGS="--teleop-mode human_mimic --elbow-hint hybrik" \
  ./quest/run_openarm_cartesian_bimanual.sh
```

HybrIK UDP 默认 `47950`；摄像头默认 `/dev/video8`（`OPENARM_HYBRIK_CAM`）。

---

## 2. 仿真双臂

```bash
cd openarm_teleop1

OPENARM_QUEST_EXTRA_ARGS="--teleop-mode human_mimic" \
  ./quest/run_openarm_cartesian_bimanual_sim.sh
```

- 无 CAN；MuJoCo 窗口 + bridge + 控制 + 录制/点动面板
- 可选 HybrIK：同真机，先 `HybrIK/run_swivel_bridge.sh`，再加 `--elbow-hint hybrik`

| | 真机 | 仿真 |
| --- | --- | --- |
| 脚本 | `./quest/run_openarm_cartesian_bimanual.sh` | `./quest/run_openarm_cartesian_bimanual_sim.sh` |
| CAN | 需要 | 无 |
| MuJoCo | 无 | 有 |
| 相机终端 | 可选 `./quest/run_quest_camera_bridge.sh` | 同左 |

---

## 3. 操作流程（通用）

1. 启动脚本，日志出现 `teleop_mode=HUMAN_MIMIC`
2. Quest 打开 HTTPS 链接，`client=connected`
3. **按住 Grip** 跟手；A/X 闭爪、B/Y 开爪
4. 录制面板：**开始录制** → 操作 → **停止录制**
5. `Ctrl+C` 或面板退出；真机确认卸力

### 夹爪

- 行程斜坡控制；真机默认 `OPENARM_GRIPPER_KP=10`；右手夹持力可调 `OPENARM_GRIPPER_GRASP_LAG_RIGHT`（Kp 默认仍为 10）；`OPENARM_GRIPPER_CLOSE_SPEED=0.45`
- 遇阻 soft-stall：`OPENARM_GRIPPER_SOFT_STALL=1`（默认）

### 肘部与 IK（当前实现）

- IK 任务：**腕 XYZ + 姿态 3 + 肘 XYZ 3**（对外是关节**位置**参考，不是速度指令）
- **Soft-reach + 下探/前伸进度**（**默认开**）：未够远弯肘，前伸或探底才接近竖直；日志 `reach_p=`（max 下探/前伸）
  - 关进度：`OPENARM_HM_REACH_SOFT_PROGRESS=0`
  - 只关前伸进度：`OPENARM_HM_REACH_SOFT_FWD=0`
  - 全关 soft-reach：`OPENARM_HM_REACH_SOFT=0` → 腕 1:1，**中段易肘崩直**
  - 肘权随进度衰减：`OPENARM_HM_REACH_ELBOW_YIELD` / `OPENARM_HM_REACH_ELBOW_MIN`
  - 详见 [`SOP.md` §6.6](../SOP.md#66-human_mimic-工作空间--soft-reach--下探进度)
- 虚拟骨架生成肘目标；HybrIK（可选）混合 φ + 歧义消解
- **伸展 yield** / **tip-priority**（`tip_y=`）在近伸展或腕误差大时调肘权
- 求解器：`OPENARM_HM_IK_BACKEND=diff_qp`（默认）或 `lm`
- 日志：`e_elb`、`phi_base` / `phi_cmd`、`reach_p`、`stretch`、`tip_y`

### 桌面面板

1. **动作录制**（UDP 47930）：会话名 → 开始/停止；退出触发 Kd 阻尼 ~6 s
2. **TCP 微调**（UDP 47931）：按住方向键点动（默认 20 mm/s）；**双手相对锁定**在面板上切换；锁定后可设**半径(mm)** 点「画圆」——以当前右臂 TCP 为圆心、水平面转一圈（默认 8 s，线速度随半径增大），左臂相对跟随；「停止」可中途取消。

---

## 4. 其它模式（索引）

| 模式 | CLI | 说明 |
| --- | --- | --- |
| `cartesian_precision` | 默认（无 `--teleop-mode`） | DLS 末端精度（二进制默认；日常请显式写 `human_mimic`） |
| `skeleton_proximal` | `--teleop-mode skeleton_proximal` | 近端 SEW + `HybrIK/run_skeleton_pose_bridge.sh` |
| `pika_style` | `--teleop-mode pika_style` | 试验：6D DIFF_QP（无肘骨架 / soft-reach） |
| `--body-pose` | + `./quest/run_body_pose_bridge.sh` | 人体关键点可视化，不驱动关节 |

### 试验：`pika_style`

```bash
OPENARM_QUEST_EXTRA_ARGS="--teleop-mode pika_style" \
  ./quest/run_openarm_cartesian_bimanual.sh
```

日志：`PIKA_STYLE (6D DIFF_QP, no elbow/soft-reach)`。权重：`OPENARM_PIKA_W_WRIST` / `W_ORI` / `W_CONT` / `W_NOM`。日常请回到 `human_mimic`。

---

## 5. 常见问题

**Quest 连不上** — 查 `8443`、UDP 47911/12、LAN IP、证书继续访问。

**真机 CAN** — `ip link show can0 can1`；`OPENARM_SKIP_CAN_CONFIG=1` 跳过自动配置。

**模式未生效** — 日志需有 `teleop_mode=HUMAN_MIMIC`；检查 `OPENARM_QUEST_EXTRA_ARGS`。

**HybrIK 无效果** — 必须 `--elbow-hint hybrik` 且 `run_swivel_bridge.sh` 在跑；看 `hint_ok=1`。

**中段肘突然崩直** — 多半是 `OPENARM_HM_REACH_SOFT=0` 或跑了 `pika_style`。保持 soft-reach 开；日志看 `reach_p` / `d_req`。不要用「取消肘限制」治崩直。

**中程回弹 / 肘跳** — 看 `tip_y`、`stretch`、`reach_p`、`lim@j*`、`flip`。默认同进度 soft-reach 已开。只关进度：`OPENARM_HM_REACH_SOFT_PROGRESS=0`；全关：`OPENARM_HM_REACH_SOFT=0`（易崩直）。

**环境变量全集** — 见 [`SOP.md` §6](../SOP.md#6-有效环境变量完整列表)。
