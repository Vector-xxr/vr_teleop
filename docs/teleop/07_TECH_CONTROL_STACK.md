# 07 — 控制栈技术说明

聚焦 `quest_cartesian_control` 真机/仿真共用的 C++ 控制链。

---

## 1. 模块表

| 模块 | 路径 | 职责 |
| --- | --- | --- |
| 入口 / UI / 启动动作 / 双臂线程 | `control/openarm_quest_cartesian_control.cpp` | 组装 bundle、播 startup、couple、面板 UDP |
| UDP 接收 | `src/io/quest_pose_receiver.*` | Quest CSV → `QuestPoseSample` |
| Human Mimic | `src/controller/human_mimic_controller.*` | clutch、映射、soft-reach、夹爪、IK 步进 |
| 配置 | `src/controller/human_mimic_types.hpp` | 默认参数（env 可覆盖） |
| 虚拟骨架 | `src/controller/virtual_arm_skeleton.*` | 肩肘腕 / φ |
| IK | `src/controller/human_mimic_ik_solver.*` | 默认 DIFF_QP 整臂 |
| 运动学 | `src/controller/human_mimic_kinematics.*` | FK / Jacobian |
| 关节参考 | `src/controller/joint_reference_generator.*` | q/qdot/qddot 限制 |
| 官方控制步 | `src/controller/control.*` | `unilateral_step` MIT 跟踪 |
| 安全层 | `src/safety/safety_filter.*` | 限幅 |
| 硬件 | `src/hardware/teleop_hardware.*` + `openarm_port/*` | CAN MIT / MuJoCo |
| 增益 | `config/follower.yaml` | 臂 + 夹爪 Kp/Kd |

构建：`cmake --build openarm_teleop/openarm_inplace/build --target quest_cartesian_control`。

---

## 2. 稳定数据流（HM 承诺）

```text
VR/面板目标
  → VirtualArmSkeleton（肘目标）
  → HumanMimicIkSolver（单一 q_target）
  → JointReferenceGenerator
  → Control::unilateral_step
  → SafetyFilter
  → Hardware
```

HybrIK / body hint **只影响肘目标**，不在 IK 之后改写关节。

---

## 3. HumanMimicController::step 摘要

1. 读实测关节 `q_meas`、最新 `QuestPoseSample`  
2. 超时 / 需重 clutch → hold  
3. **`couple_follow`**：跳过 VR，只 `advance_joint_stream(q_target_)`  
4. 无 grip：可 panel jog；否则 hold  
5. Grip 上升沿 → `activate_clutch`（锚点、φ 种子、soft-reach progress）  
6. 映射 `Δp` / 姿态 → 腕目标 → `solve_base_skeleton`  
7. 夹爪按钮状态机 → gripper target  
8. 输出 reference  

关键连续性：

- `q_target_slew`：避免单拍跳解  
- `elbow_cmd_phi_` slew：避免肘支路突变  
- clutch 后 idle：手未离开死区前不跟 IK  

---

## 4. 控制频率与线程

- 参考线程：收 Quest、跑 HM、写 reference（双臂时可左右交替；couple 时右→左）  
- 控制线程：`unilateral_step` ~1 kHz 跟踪 reference  

具体 Hz 以 `FREQUENCY` / 定时器实现为准（见 `openarm_quest_cartesian_control.cpp`）。

---

## 5. 安全与增益

- `SafetyFilter`：非有限值、过速等拦截  
- 真机 Quest 可用 `OPENARM_QUEST_MIT_SCALE` 软化增益；**回放不用**  
- 夹爪：MIT Kp/Kd 与 grasp-hold 相对滞后（`OPENARM_GRIPPER_*`）  

---

## 6. 与回放栈共用

`dual_replay_control` 在 reference 源换成 CSV 后，**下游与上表相同**（Control → Safety → Hardware）。  
这是「录一次、回放多次」能复现的基础。
