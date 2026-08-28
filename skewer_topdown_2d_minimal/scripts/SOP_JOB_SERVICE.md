# 常驻任务服务：启动动作一次 + 按命令执行多套 action1→2D抓取→action2

把原来的「一启动就跑完 02→抓取→03 然后退出」封装成：

1. **服务常驻**：一个双臂 `dual_replay_control` 进程一直开着  
2. **启动时只做一次开始动作**（`recordings/tasks/steady_action_on_table.csv`）  
3. **平时空闲**，不自动跑抓取链路  
4. **收到命令**（如 `1`）才执行 `recordings/job1` 整套：`action1 → 2D抓取 → action2`  
5. 支持多个任务槽：`recordings/job1`、`job2`、`job3` …

旧入口 `run_quest_02_align_03.sh` 仍保留（一次性跑完即退）。

---

## 目录约定

服务默认直接读（**不再依赖** `skewer.../jobs/1` 的 symlink）：

```text
<vr_control>/openarm_teleop/openarm_inplace/recordings/
  tasks/steady_action_on_table.csv   # 启动动作
  job1/
    action1.csv   # 命令 "1" / "job1"
    action2.csv
    job.json      # 可选
  job2/
    action1.csv   # 命令 "2" / "job2"
    action2.csv
    job.json
  job3/
    ...
```

桥接临时文件写在：

```text
skewer_topdown_2d_minimal/jobs/_runtime/
```

**启动动作（steady）默认路径**（与旧现场一致）：

```text
<vr_control>/openarm_teleop/openarm_inplace/recordings/tasks/steady_action_on_table.csv
```

也可用环境变量覆盖：

```bash
export OPENARM_STARTUP_CSV=/abs/path/to/steady_action_on_table.csv
```

每个任务文件夹至少要有 **双臂 merged** 的 `action1.csv` + `action2.csv`（含 `q0..q15`）。

桥接规则与旧脚本相同：

- 用 action1 **最后一帧右臂**作为 2D 观测起始  
- 抓完后先到 action2 **第一帧右臂**，再播 action2  

---

## 启动服务

```bash
cd /home/lan/openarm_teleop/vr_control/skewer_topdown_2d_minimal
conda activate anygrasp   # 或你现场环境

OPENARM_SUDO_PASSWORD=你的密码 \
OPENARM_RIGHT_CAMERA_INDEX=16 \
scripts/run_skewer_job_service.sh
```

日志出现类似：

```text
[skewer-job] playing startup action once
[skewer-job] ready. Send UDP to :47925 ...
[skewer-job] available jobs: ['1']
```

之后服务空闲等待命令。

常用环境变量：

| 变量 | 默认 | 含义 |
|------|------|------|
| `OPENARM_SKEWER_JOB_LISTEN_PORT` | 47925 | 任务命令端口 |
| `OPENARM_QUEST_ALIGN_COMMAND_PORT` | 47920 | 回放控制端口 |
| `OPENARM_QUEST_ALIGN_STATUS_PORT` | 47921 | 回放状态端口 |
| `OPENARM_LEFT_FOLLOWER_CAN` | follower_left | 左臂 CAN |
| `OPENARM_RIGHT_FOLLOWER_CAN` | follower_right | 右臂 CAN |
| `OPENARM_RIGHT_CAMERA_INDEX` | 16 | 右腕相机 |
| `OPENARM_GRASP_DESCENT_M` | 0.04 | 下探深度 |

---

## 发命令

另开终端：

```bash
cd /home/lan/openarm_teleop/vr_control/skewer_topdown_2d_minimal

# 查询状态 / 已发现任务
python3 scripts/skewer_job_client.py status

# 执行任务 1（action1→2D→action2）
python3 scripts/skewer_job_client.py 1

# 执行任务 2 / 3（文件夹准备好后）
python3 scripts/skewer_job_client.py 2
python3 scripts/skewer_job_client.py 3

# 中止当前任务（尽量打断 play/grasp）
python3 scripts/skewer_job_client.py abort

# 结束服务（会 shutdown 回放进程）
python3 scripts/skewer_job_client.py shutdown
```

也可用原始 UDP（任意语言）：

```bash
# 示例：发 "1"
python3 - <<'PY'
import socket
s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2)
s.sendto(b"1", ("127.0.0.1", 47925))
print(s.recvfrom(4096)[0].decode())
PY
```

回复示例：

```text
accepted|job=1|from=127.0.0.1:xxxxx
busy|job=1
ok|busy=0|job=-|startup_done=1|last=success|error=-|jobs=1
error|unknown_job|2
```

---

## 安全 / 鲁棒性

| 机制 | 行为 |
|------|------|
| 单任务互斥 | 执行中再发命令 → `busy`，不并行抢 CAN |
| 启动门闩 | startup 未完成前拒绝任务 |
| 预检 | 缺 CSV / 列不对 → 直接失败，不开始动臂 |
| 抓取失败默认跳过 action2 | 避免「没抓到还继续后半段」；可用 `--continue-action2-on-grasp-fail` 关闭 |
| abort | 置位后打断等待；grasp 子进程会被 terminate |
| 回放异常 | `load_failed` / `safety` / 超时 → 任务失败并记入 `last_error` |
| 控制进程挂掉 | 服务检测到后退出，避免假空闲 |
| 退出清理 | SIGINT/SIGTERM/shutdown → 给回放发 `shutdown` |

现场仍建议：急停在手边；不要同时开 VR 遥操或其它占用同一 CAN 的进程。

---

## 添加任务 2 / 3

```bash
REC=openarm_teleop/openarm_inplace/recordings

# 放入双臂 merged CSV
cp /path/to/before.csv "$REC/job2/action1.csv"
cp /path/to/after.csv  "$REC/job2/action2.csv"

# 可选覆盖
cp "$REC/job1/job.json" "$REC/job2/job.json"   # 再按需改相机/下探
```

无需重启服务即可被 `status` 发现（每次命令时扫描目录）。若正在跑任务，等结束后再触发。

---

## 与旧 02→03 脚本关系

| | 旧 `run_quest_02_align_03.sh` | 新 job service |
|--|--|--|
| 启动 | 立刻跑完整套后退出 | 播 startup 后空闲 |
| 多任务 | 改环境变量/路径 | `recordings/job1` `job2` … |
| 触发 | 无 | UDP `1/2/3` |
| 控制进程 | 一套完结即关 | 常驻，多任务复用 |

原 SOP（`SOP_QUEST_02_ALIGN_03.md`）仍可用于一次性调试。
