# 09 — 主从锁定与面板点动（技术）

实现集中在：

- `control/openarm_quest_cartesian_control.cpp` → `QuestUiBridge`  
- `src/controller/human_mimic_controller.*`  

流程入口见 [03_PIPELINE_PANELS.md](03_PIPELINE_PANELS.md)。

---

## 1. 主从锁定（Couple）

### 1.1 Engage 快照

```text
couple_R_right = R_r                          # 冻结
couple_p_rel   = R_r^T (p_l - p_r)            # 右 TCP 系下相对平移
couple_R_rel   = R_r^T R_l
右/左 set_orientation_lock
左 set_couple_follow(true)  → 种子 elbow_cmd_phi_，couple_phi_lock_
```

因双臂姿态锁死，运行期：

```text
p_L_des = p_R_cmd + couple_R_right * couple_p_rel
        = p_R_cmd + (p_L0 - p_R0)             # 机体系固定 Δp
R_L_des = couple_R_right * couple_R_rel       # 恒定
```

### 1.2 每拍

1. `step_arm(right)` — VR 仅位置（姿态锁）  
2. `apply_couple_left_follow(dt)` — 几何目标 ± 保守 PI → `drive_tcp_robot`  
3. `step_arm(left)` — `couple_follow`：只 `advance_joint_stream`  

### 1.3 连续性（防换解 / 突变）

| 机制 | 作用 |
| --- | --- |
| `couple_phi_lock_` | 拒绝 `{φ, φ+π}` 翻支；φ 仅慢速连续 slew |
| `continuous_tip_drive_` | tip chase **保留 `q_target_slew`**，不用全量 `jog_active` 跳解 |
| soft-reach progress→1 | 减少过期弯臂裕度压短相对偏置 |
| JRG 提高 k/qdot，不盲目 Reset | 跟上主臂同时避免把跳变打进电机 |
| PI 默认 | `CORR_MAX` 默认 **6 cm**，`KP/KI`≈0.4/0.7（仍低于旧 8 cm / 0.55/1.2） |
| Chase / JRG | 默认 `CHASE_ITERS=8`，`K_SCALE=12`，`QDOT_SCALE=8` |
| 内扣抬肘滞回 | `elbow_raise_hold_*`：近身抬肘后慢放 φ / 弱化 medial→FK；`ELBOW_Q_SLEW_DOWN` 使肘屈打开更慢；medial 区淡化 droop 评分 |
| 腕限位逃生 | `wrist_escape_phi_*`：j4–j6 顶死且 `e_ori` 大时解冻 φ、少软化 ori，让近端腾出腕行程 |

可选 env：`OPENARM_COUPLE_CHASE_ITERS`、`OPENARM_COUPLE_K_SCALE`、`OPENARM_COUPLE_QDOT_SCALE`、以及 `OPENARM_COUPLE_*` FB 组；肘：`OPENARM_HM_ELBOW_Q_SLEW_DOWN`、`OPENARM_HM_ELBOW_RAISE_HOLD`。

### 1.4 日志

约 1 Hz：`[quest-ui] couple-fb |res|=… |vr|=… |vl|=… phi_flip=… e_w=…`  
期望跟随良好时 `phi_flip=0`，`|vl|` 接近 `|vr|`，`|res|` 多在死区内。

---

## 2. 面板点动

### 2.1 轴向

C++ `jog_delta` **硬编码**机体轴（不经 `up×forward`，避免 `OPENARM_HM_FORWARD_SIGN` 静默对调左右）：

```text
up=(0,0,1)  down=(0,0,-1)  left=(0,1,0)  right=(0,-1,0)
```

### 2.2 Hold 跟踪

```text
set_panel_jog_velocity(v_body)
  → apply_panel_jog_tick:
       加速度限制 |v|
       panel_tcp_des_ += v*dt
       continuous_tip_drive_ + solve_base_skeleton chase
       Reset(JRG) 输出（q 已经过 slew，安全）
```

旧版速度 DLS 在限位附近易斜出轴；现改为笛卡尔目标 chase。

### 2.3 速度

权威：`OPENARM_QUEST_JOG_SPEED`（m/s）。面板 `--speed` 仅 UI。

---

## 3. 画圆

- 右臂 `set_couple_follow(true)` 临时跳过 VR，由 `drive_circle_tcp` 驱动  
- 轨迹在机器人 **XY**；Z 保持圆心高度  
- 径向 ramp → 轨道 → 一圈结束 `stop_circle`  
- 左臂仍走 couple 跟随  

---

## 4. 回滚提示

主从/点动大改前备份分支示例：`backup/pre-couple-jog-fix-*`。  
工作分支示例：`fix/couple-jog-continuity`。
