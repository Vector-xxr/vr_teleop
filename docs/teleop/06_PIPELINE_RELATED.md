# 06 — 相关旁路链路（非主 VR 遥操）

下列路径与 OpenArm 运动有关，但**不是**「Quest 手柄 → human_mimic」日常主链。列入本目录是为了全链路盘点时不混淆。

---

## 1. 视觉抓取 / Skewer Topdown

路径示例：

- `skewer_topdown_2d_minimal/`（仓库根或 `openarm_inplace/` 下）
- `openarm_teleop/openarm_inplace/left_grasp_exec_bundle/`
- 编排：`openarm_inplace/script/run_quest_02_align_03.sh`
- SOP：`skewer_topdown_2d_minimal/SOP_QUEST_02_ALIGN_03.md`、`SOP_FULL.md`

特征：

- **不经过** Quest WebXR clutch；常驻或一次性拉起 `dual_replay_control`（UDP **47970/47971**）
- **消费**遥操录制的 CSV（如 `sessions/quest/02|03`、`tasks/action1.csv`），再叠加视觉对齐/抓取
- 典型编排：播 action1 → 视觉对齐抓取 → 播 action2

与 VR 遥操关系：共用机械臂与回放基础设施；**控制进程互斥**（勿与 `quest_cartesian_control` 同时抢 CAN）。

Vendor 拷贝：`*/vendor/openarm_inplace/` 为打包副本，**以主树 `openarm_inplace/src` 与 `script/` 为准**。

---

## 2. Leader–Follower 硬件主从录制（非 Quest）

- `script/run_cpp_real_teleop_record(_both).sh`：物理 leader + follower
- `script/run_cpp_real_follower_record*.sh`：仅 follower
- 产物仍落在 `recordings/sessions/`，可用 §05 回放链复现
- 官方网站文档里的 leader-follower 与本仓库 Quest 从臂方案不同

---

## 3. Fairino（quest/fairino）

- `quest/fairino/Robot.py` 等：法奥等外臂 Python SDK 遗留  
- `quest/server.py` / `run_quest.py`：早期 Quest→Fairino（XML-RPC），**不是** OpenArm CAN  
- `run_simple.py` 依赖可能缺失，视为破损遗留  

**当前 OpenArm 产品路径不依赖 Fairino。** Bridge 用 `quest_bridge.py`。

---

## 4. 其它测试入口

| 脚本 | 用途 |
| --- | --- |
| `quest/test_teleop_trajectory.sh` | 轨迹/遥操冒烟 |
| `quest/test_gripper_chain.sh` | 夹爪链 |
| `openarm_inplace/script/run_cpp_real_gripper_test.sh` | 真机夹爪 + 回放链 |
| `quest/test_camera_fps.py` | 相机帧率 |
| `script/launch_{grav_comp,unilateral,bilateral}.sh` | 官方无 Quest 控制入口 |

---

## 5. 官方 openarm 网站文档

`openarm_teleop/openarm/website/docs/teleop/leader-follower/*` 描述官方主从机械臂遥操（物理 leader），与本仓库 **Quest VR 从臂** 方案不同，勿混用配置。
