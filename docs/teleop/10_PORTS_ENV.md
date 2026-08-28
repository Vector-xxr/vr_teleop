# 10 — 端口与环境变量速查

完整大表见 [SOP.md](../../SOP.md) §6。此处只列遥操链路最常用项。

---

## 1. 端口

| 端口 | 协议 | 用途 |
| --- | --- | --- |
| **8443** | HTTPS | Quest `client.html` / WebXR 静态页 |
| **8442** | WSS | Quest 姿态 WebSocket（bridge 内） |
| **8444** | HTTPS | FPV MJPEG `/cam.mjpg` |
| **47911** | UDP | 右手柄 → 控制 |
| **47912** | UDP | 左手柄 → 控制 |
| **47930** | UDP | 录制面板 → 控制（**勿与双臂回放 start 口并发**） |
| **47931** | UDP | 点动/主从面板 → 控制（同上） |
| **47940** | UDP | 回放扭矩遥测面板（可选） |
| **47945** | UDP | 夹爪遥测 → Bridge HUD（本机） |
| **47950** | UDP | HybrIK 肘 φ hint（`--elbow-hint`） |
| **47951** | UDP | HybrIK SEW（`skeleton_proximal`） |
| **47952** | UDP | Body Pose 关键 → 控制（试验） |
| **47970/47971** | UDP | `dual_replay_control` 常驻服 cmd/status |

覆盖示例：`OPENARM_QUEST_HTTP_PORT`、`OPENARM_QUEST_CAMERA_PORT`、`OPENARM_QUEST_UI_*_PORT`、`OPENARM_BODY_POSE_PORT`。

---

## 2. 启动与模式

| 变量 | 常用值 | 说明 |
| --- | --- | --- |
| `OPENARM_QUEST_EXTRA_ARGS` | `--teleop-mode human_mimic` | **必带**才走产品 HM |
| `OPENARM_QUEST_STARTUP_FILE` | `…/tasks/steady_action_on_table.csv` | 启动轨迹 |
| `OPENARM_QUEST_UI` | `1` | 录制+点动面板 |
| `OPENARM_QUEST_CONDA_ENV` | `openarm_quest` | Python 环境 |
| `OPENARM_TELEOP_URDF` | `…/v1.urdf` | URDF |

---

## 3. 点动 / 主从

| 变量 | 默认（现） | 说明 |
| --- | --- | --- |
| `OPENARM_QUEST_JOG_SPEED` | `0.02` | 点动尖端速度 m/s |
| `OPENARM_QUEST_JOG_STEP` | `0.01` | 旧 nudge 步长 m |
| `OPENARM_COUPLE_FB` | `1` | 残差闭环 |
| `OPENARM_COUPLE_KP` / `KI` | `0.25` / `0.4` | 保守 PI |
| `OPENARM_COUPLE_CORR_MAX` | `0.03` | 校正上限 m |
| `OPENARM_COUPLE_DEADBAND` | `0.008` | 死区 m |
| `OPENARM_COUPLE_CHASE_ITERS` | `3` | 从臂 IK chase |
| `OPENARM_COUPLE_K_SCALE` | `8` | 从臂 JRG k |
| `OPENARM_COUPLE_QDOT_SCALE` | `5` | 从臂 JRG qdot |
| `OPENARM_QUEST_CIRCLE_PERIOD` | `8` | 画圆周期 s |

---

## 4. Human Mimic（高频）

| 变量 | 说明 |
| --- | --- |
| `OPENARM_HM_POS_WORLD` | `1`：VR 世界系抬升→机体 +Z |
| `OPENARM_HM_REACH_*` | soft-reach 裕度/进度 |
| `OPENARM_HM_CLUTCH_PHI_HOLD` | 默认 `0`：关肘硬锁阶跃 |
| `OPENARM_HM_Q_SLEW*` / `ELBOW_*` | 关节/肘连续性 |
| `OPENARM_HM_QDOT_MAX` | JRG 角速度上限（默认 6） |
| `OPENARM_HM_K_JOINT` | JRG 增益（默认 14） |
| `OPENARM_HM_FORWARD_SIGN` | 工作空间「前」符号；**不再改面板左右** |

---

## 5. 硬件 / 增益

| 变量 | 说明 |
| --- | --- |
| CAN 名 | `follower_left` / `follower_right`（compat env） |
| `OPENARM_QUEST_MIT_SCALE` | Quest 真机软增益；回放忽略 |
| `OPENARM_GRIPPER_*` | 夹爪行程与 grasp-hold |

---

## 6. 相机 / Body / HybrIK

| 变量 | 说明 |
| --- | --- |
| `OPENARM_QUEST_CAMERA` | 默认 `16` → `/dev/video16` |
| `OPENARM_QUEST_CAMERA_PREVIEW` | PC 预览窗 |
| `OPENARM_BODY_POSE_BACKEND` | `hybrik` / `mediapipe` |
| `OPENARM_HYBRIK_CAM` | Body 相机号 |

相关试验桥（驱动肘提示，非日常手柄路径）：

- 肘 φ：`openarm_teleop1/.../HybrIK/run_swivel_bridge.sh` → **47950** + `--elbow-hint hybrik`
- SEW：`run_skeleton_pose_bridge.sh` → **47951** + `--teleop-mode skeleton_proximal`

## 7. 端口冲突注意

| 冲突 | 说明 |
| --- | --- |
| 47930/47931 | Quest UI 与部分回放 start 共用语义 → **不要** VR 遥操与 dual replay 同时开 |
| 47950/47951 | HybrIK 与部分 leader 录制 monitor 可能撞车 |
| CAN 命名 | 双臂产品用 `follower_left/right`；单臂脚本默认可能是 `can0/can1` |
