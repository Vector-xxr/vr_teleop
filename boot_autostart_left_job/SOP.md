# SOP：编排服务开机自启

工程根：`/home/knowin/Desktop/vr_control`（若不同请全文替换路径）。

---

## 0. 安装前检查

```bash
# 手动能跑通再做自启
cd /home/knowin/Desktop/vr_control/left_job_orchestrator
export OPENARM_SUDO_PASSWORD=knowin   # 改成你的
./run_service.sh
# Ctrl+C 停掉后再装 systemd
```

确认：

- [ ] CAN：`follower_left` / `follower_right`
- [ ] 轨迹：`recordings/job1/`、`job2/` 等齐全
- [ ] 现场有急停；开机自动起服务会动臂

---

## 1. 准备环境文件

| 源文件（本包） | 目标路径 |
| --- | --- |
| `boot_autostart_left_job/env.systemd.example` | `left_job_orchestrator/env.systemd` |

```bash
cd /home/knowin/Desktop/vr_control/boot_autostart_left_job
cp env.systemd.example ../left_job_orchestrator/env.systemd
chmod 600 ../left_job_orchestrator/env.systemd
```

编辑 `../left_job_orchestrator/env.systemd`，至少设置：

- `OPENARM_SUDO_PASSWORD` — 本机 sudo 密码（CAN 配置用）
- `OPENARM_LEFT_JOB_API_TOKEN` — 与对接方约定的 token（默认示例可改）
- `OPENARM_ROS_SETUP` — 目标机 ROS setup，默认 `/opt/ros/humble/setup.bash`

该文件含密码，**不要 git add / commit**（已在仓库 `.gitignore` 忽略）。

---

## 2. 安装 systemd 单元

| 源文件（本包） | 目标路径 |
| --- | --- |
| `boot_autostart_left_job/openarm-left-job.service` | `/etc/systemd/system/openarm-left-job.service` |

方式 A — 脚本（推荐，可换路径/机器）：

```bash
cd /path/to/vr_control/boot_autostart_left_job
# 安全默认：只安装，不 enable、不启动
sudo ./install.sh
# 改目标机密码/token
nano ../left_job_orchestrator/env.systemd
# 普通用户完成迁移编译/验证
../migration_build/build_all.sh --clean
../migration_build/build_all.sh --check
# 启用开机启动（仍不立即启动）
sudo ./install.sh --enable
# 清除历史 failed 状态（不会启动）
sudo systemctl reset-failed openarm-left-job
# 现场安全后才立即启动
sudo systemctl start openarm-left-job
```

`install.sh` 会按**当前仓库路径**和 `sudo` 用户自动改写 service 里的路径/User，并固定该用户的 `openarm_teleop1` Python。非标准 conda 路径使用 `--python /绝对路径/bin/python`。`--enable` / `--start` 会检查密码、Python、ROS setup，并在加载 ROS 后对 vendor binary 做动态库检查；这些检查不访问硬件。`--start` 可安装后立即启动并执行 steady（可能动臂），不建议首次迁移使用。

方式 B — 手动（不推荐，仅本机固定路径时）：

```bash
sudo cp /home/knowin/Desktop/vr_control/boot_autostart_left_job/openarm-left-job.service \
  /etc/systemd/system/openarm-left-job.service
sudo systemctl daemon-reload
sudo systemctl enable openarm-left-job.service
sudo systemctl start openarm-left-job.service
```

---

## 3. 验证

```bash
sudo systemctl status openarm-left-job.service
journalctl -u openarm-left-job -n 50 --no-pager

# 本机测端口（服务起来且 steady 播完后）
export PY=/home/knowin/miniconda3/envs/openarm_teleop1/bin/python
cd /home/knowin/Desktop/vr_control
$PY -m left_job_orchestrator.test_socket --host 127.0.0.1 status
```

日志里应出现类似：`ready. TCP :47925`。

---

## 4. 路径改过怎么办

无需手改模板。把整份仓库放到目标路径后，从该仓库运行 `sudo ./install.sh`，脚本会自动渲染：

- `User=` / `Group=`
- `WorkingDirectory=`
- `EnvironmentFile=`
- `ExecStartPre=` / `ExecStart=`

只修改 `env.systemd` 不需要 `daemon-reload`，下次启动会重新读取。修改 unit
模板、仓库路径或部署脚本后，重新运行 `sudo ./install.sh --enable`；安装脚本会
执行 `daemon-reload`，但不会启动服务。

---

## 5. 卸载 / 取消自启

```bash
cd /home/knowin/Desktop/vr_control/boot_autostart_left_job
sudo ./uninstall.sh
```

或：

```bash
sudo systemctl disable --now openarm-left-job.service
sudo rm -f /etc/systemd/system/openarm-left-job.service
sudo systemctl daemon-reload
```

`left_job_orchestrator/env.systemd` 需自行决定是否删除。

---

## 6. 常见问题

| 现象 | 处理 |
| --- | --- |
| 开机未启动 | `systemctl is-enabled openarm-left-job`；看 `journalctl -u openarm-left-job -b` |
| CAN / sudo 失败 | 检查 `env.systemd` 里密码；确认 CAN 接口名 |
| CAN 等待超时 | 检查 USB-CAN/udev 接口；可用 `OPENARM_CAN_WAIT_TIMEOUT_S` 调整默认 60 秒 |
| 找不到 python | 确认 `~/miniconda3/envs/openarm_teleop1`；或在 env 里设 `OPENARM_PYTHON=` |
| `librcutils.so` / ROS 库找不到 | 在 env 中把 `OPENARM_ROS_SETUP` 设为目标机实际的 `setup.bash`，再运行 `sudo ./install.sh --enable` |
| `ExecStartPre` 失败 | 普通用户运行 `migration_build/build_all.sh --check`；缺目标时先 `--clean` 编译 |
| 与 VR 冲突 | 勿同时开 VR / `run_once` / 本服务 |
| 想延迟再动臂 | 在 service 的 `[Service]` 增加 `ExecStartPre=/bin/sleep 30` 后重新 `install.sh` |

启动失败后：

```bash
cd /home/knowin/Desktop/vr_control/boot_autostart_left_job
nano ../left_job_orchestrator/env.systemd
sudo ./install.sh --enable
sudo systemctl reset-failed openarm-left-job
journalctl -u openarm-left-job -b -n 100 --no-pager
# 确认其它控制链已停、急停可用且工作区安全后，才执行：
sudo systemctl start openarm-left-job
```
