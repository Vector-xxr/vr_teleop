# action1 → 五次多项式到 begin → 固定夹爪框抓取 → 五次多项式到 action2

## 环境配置

```bash
cd /home/knowin/Desktop/vr_control/tcp_forward_grasp

# 需要 conda openarm_teleop1（Python 3.11）；run_once.sh 会优先选用
conda activate openarm_teleop1

export OPENARM_SUDO_PASSWORD=knowin   # CAN 配置；按本机密码修改

# 电机/夹爪与 VR 一致（dual_replay server → openarm_vr_compat_env.sh）
```

## 默认抓取（已接入动作轨迹）

```bash
./run_once.sh
```

流程：

0. **准备动作** `recordings/tasks/steady_action_on_table.csv`（与 VR 遥操启动相同）
1. 播放 `action1.csv`
2. **五次多项式**平滑到 `begin.csv` 末帧（抓取初始位 + 开爪）
3. 固定夹爪框 + 锁定**最右红色手柄**（整段红珠+糖针）→ 左右对齐 → **流式 TCP 逼近**；红框与夹爪门重叠 **≥60%** 即停并闭合 → 抬起 → **后退约 14cm**
4. **五次多项式**平滑到 `action2.csv` 首帧
5. 播放 `action2.csv` → 保持

跳过准备动作：`./run_once.sh --skip-steady`  
只抓取、不跑动作轨迹：`./run_once.sh --grasp-only`  
调试只做左右对齐：`./run_once.sh --lr-only`

## 白夹爪框（已离线标定）

固定文件：`config/gripper_gate_left.json`（`y_tip=279`）。重新标定：

```bash
./run_calibrate_gate.sh          # 只张开夹爪，不开 begin 整段
./run_calibrate_gate.sh --camera-only
```

## 节奏（Phase2 = 独立流式逼近伺服）

逼近**不再**走 `play|csv` 微步；Python ~25Hz 发 `tcp_vel`，C++ `dual_replay_control` 内 1kHz 加速度限制积分 TCP→IK→MIT。

| 项 | 约 |
|----|----|
| LR 单步 | 2.2s + settle 0.22s |
| 前伸 | **匀速流式**：默认 25mm/s；横向 ~0.5 mm/s/px；指令超时 ~0.15s；红球进入开口即 `approach_stop` |
| 相机预览 | **默认开启**（逼近全程刷新） |
| 闭爪 | 0.9s + hold 0.5s |
| 抬起 / 后退 | 各 ~1.0s；后退 140mm |
| action1→begin / grasp→action2 五次多项式 | 各 3.0s |

可用默认项：`approach_speed_mm_s`、`approach_lat_gain_mm_s_per_px`、`approach_accel_mm_s2`、`approach_cmd_timeout_s`。  
也可用 `--pre-to-observe-duration` / `--post-replay-transition-duration` / `--post-grasp-retreat-mm` 覆盖编排。

日志：`tcp_forward_grasp/outputs/action_grasp_action_*/` 或 `lr_fwd_close_*/`  
逼近段应看到 `approach_on` / `tcp_vel` / `approach_stop`，不应再有 `fwd-lr-N` CSV。
