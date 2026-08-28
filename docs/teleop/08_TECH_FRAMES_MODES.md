# 08 — 坐标系、模式与行为语义

---

## 1. 坐标系

### 1.1 Quest

- WebXR **local-floor**，通常 **+Y 向上**  
- Bridge 转发手柄位姿（位置 + 四元数）与按键  

### 1.2 机器人

- FK 根：`openarm_body_link0`（与 world 对齐）  
- 站立认知：**+X 前、+Y 左、+Z 上**（右手系）  
- 左右臂安装相对 body 有 ±90° roll，但控制器统一在 body 系表达 TCP  

### 1.3 位置映射（默认）

`OPENARM_HM_POS_WORLD=1`：

1. 每次 grip 重锚位置零点（增量）  
2. `Δp_vr` 在 **VR 世界系**计算（与手柄倾斜无关）  
3. 经固定矩阵 `M`（`vr_to_robot_axes`）映到机体  

因此 VR 上抬 ≈ 机体 **+Z**。

`POS_WORLD=0` 时用手柄姿态反变换（旧 body 系增量，抬手易斜）。

### 1.4 姿态映射

默认 **incremental + 重力锁定**：

1. grip 重锚姿态  
2. 世界系相对转动，主要保留绕竖直轴 heading  
3. 不是手柄怎么歪腕子怎么歪  

可选：`--orientation-mapping absolute`。

---

## 2. Soft-reach（弯臂策略）

人臂行程常长于机器人。用 `reach_progress`（下压/前推意图）抬高肩→腕距离天花板：

| progress | 行为 |
| --- | --- |
| ≈0 | 保留弯臂裕度（`bent_margin`），利微调 |
| →1 | 接近近直，仍留极小 soft margin，避肘奇异 |

原则：**宁可够不到，也不换解突变**（φ / `q_target` slew）。  
默认 `OPENARM_HM_CLUTCH_PHI_HOLD=0`（不做 clutch 肘硬锁解锁阶跃）。

参数组：`OPENARM_HM_REACH_*`、`OPENARM_HM_ELBOW_*`、`OPENARM_HM_Q_SLEW*` → SOP §6。

---

## 3. 遥操模式对比

| 模式 | 肘 | soft-reach | 日常 |
| --- | --- | --- | --- |
| `human_mimic` | 虚拟骨架 φ/XYZ | 有 | **是** |
| `cartesian_precision` | 无 HM 肘策略 | 无 | 否（二进制默认！） |
| `pika_style` | 关闭肘任务 | 弱 | 试验 |
| `skeleton_proximal` | 近端骨架 | 视配置 | 试验 |

启用：`OPENARM_QUEST_EXTRA_ARGS="--teleop-mode human_mimic"`。

---

## 4. 夹爪状态机（摘要）

- 右：**A 闭 / B 开**；左：**X 闭 / Y 开**  
- 按住闭合：向关闭方向走；卡住 → **grasp-hold**（相对滞后力矩）  
- **张开键解除** grasp-hold  
- 与「双键 recenter 姿态」互斥优先级：recenter 组合优先  

细节：`quest/SOP_QUEST_HUMAN_MIMIC.md`、SOP §夹爪。

---

## 5. Clutch 语义

| 用户动作 | 行为 |
| --- | --- |
| Grip 按下 | 锚零点；可 idle 至手离开死区 |
| Grip 按住 | 增量跟手 |
| Grip 松开 | 冻结当前 reference（抗重力 hold） |
| 超时丢包 | 进入超时 hold，常需重 clutch |

面板点动可在无 grip 时移动 TCP；有 grip 时平移 clutch 原点。
