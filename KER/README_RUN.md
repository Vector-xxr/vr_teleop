# KER 链路跑通说明

**公共机部署 / Cursor 对齐清单：** 见 [DEPLOY_PUBLIC.md](DEPLOY_PUBLIC.md)。

## 实机 KER 遥操（当前）

映射：
- `swap_j67=True`（腕部两轴对调：OA j6←KER ch7，OA j7←KER ch6）
- `negate_j7=True`（真机末端上下 = swap 后的 joint7 取反）
- `negate_j6=False`
- `invert_left_grip=True`（左夹爪开合方向与右一致：常态开、捏合关）

启动：先保持当前姿态约 0.8s → 限速追 KER 约 4s（**满增益咬住慢命令**）→ 定时进入正常跟踪（**不被腕部误差卡住**）。

软启动只限速命令，不硬切；跟随时用满增益，避免慢速发飘。

```bash
cd /home/lan/openarm_teleop1/KER
./run_ker_real_teleop.sh
```

会自动弹出 **录制面板**：点「开始录制 / 停止录制」。  
**关闭录制面板（或点「退出（停止控制）」）会一并停止遥操进程。**

夹爪默认开启 **grasp-hold**：捏合遇阻后自动夹持保持（力矩≈Kp·lag），再张开 KER 夹爪即可松开；臂关节仍是连续跟随。关闭：`./run_ker_real_teleop.sh --no-grip-grasp-hold`。

数据（录=控，控制环频率 ~200Hz，**不上采样**）：

`KER/recordings/sessions/<会话>/<段号>/real_teleop_dual_merged.csv`

列格式与 Quest 相同（`time,q0,dq0,tau0,...`），但行率是 KER 控制环真实拍数。  
**回放请用 KER 专用脚本**（按 CSV 时间戳逐步下发，不做插值）：

```bash
./run_ker_replay.sh recordings/sessions/<会话>/<段>/real_teleop_dual_merged.csv
# 或目录：
./run_ker_replay.sh recordings/sessions/<会话>/<段>
# 也支持旧版 1.csv（自动列映射，不插值）：
./run_ker_replay.sh recordings/sessions/ker_xxx/1.csv
```

不要用 Quest `run_cpp_real_replay_panel.sh` 播 KER 段（1kHz 环会对 ~5ms 一格产生明显阶梯卡顿）。

旧版 `1.csv`（L_cmd/R_cmd/obs）可做列转换（不插值加密）：

```bash
python convert_ker_csv_to_quest.py recordings/sessions/ker_xxx/1.csv
```

单独开面板（若用了 `--no-panel`）：

```bash
./run_ker_record_panel.sh
```

停止遥操：`pkill -f ker_real_teleop.py`

用现成 conda 环境 `openarm_teleop1` + 仓库 MuJoCo 场景，只做关节角转换对接：

```bash
cd /home/lan/openarm_teleop1/KER
# 若 USB 权限不足（Access denied）：
#   sudo chmod a+rw /dev/bus/usb/$(lsusb -d 303a:4002 | awk '{print $2"/"$4}' | tr -d ':')

./run_ker_mujoco_sim.sh
# 或无 USB 时先验窗口：
./run_ker_mujoco_sim.sh --dummy
```

- 脚本：`ker_mujoco_teleop.py`（KER USB → `scene_table.xml`）
- 映射与 Enactic `KerPoseProcessor` 一致：度→弧度，左右臂各 7 轴 + 夹爪
- 夹爪再映射到 MuJoCo finger slide `[0, 0.044]`（启发式，可能要微调）

看窗口：动 KER，仿真双臂是否同向、同关节跟随。确认后再上真机 CAN。

---

## dora 采集栈（以后再用）

```bash
./run_ker_record.sh ker-mujoco   # dora 版 KER→MuJoCo（需额外依赖）
./run_ker_record.sh ker-nocam    # 真机无相机
```
