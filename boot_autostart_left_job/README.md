# OpenArm 编排服务开机自启（systemd 部署包）

本目录是**待安装的配置包**。`install.sh` 会按**当前仓库所在路径**生成 systemd 单元，可拷到其他 Linux 机器一键安装。

> 安全默认：直接执行安装脚本只写入单元，不 enable、不启动。只有显式 `--enable` / `--start` 才会生效。

---

## 文件一览

| 本目录文件 | 安装后位置 | 说明 |
| --- | --- | --- |
| `openarm-left-job.service` | 模板 → 渲染写入 `/etc/systemd/system/openarm-left-job.service` | 路径/用户由 `install.sh` 自动替换 |
| `env.systemd.example` | → `<仓库>/left_job_orchestrator/env.systemd` | 密码等；gitignore，勿提交 |
| `check_service_env.sh` | 原地由 systemd 调用 | 只读检查密码、Python、ROS setup、构建产物和动态库 |
| `wait_for_can.sh` | 原地由 systemd 调用 | 最多等待 USB-CAN 接口 60 秒，不配置 CAN |
| `install.sh` | 本目录执行 | 一键安装 |
| `uninstall.sh` | 本目录执行 | 停用并删除单元 |
| `SOP.md` | 本目录 | 详细步骤 |

---

## 本机最短步骤

```bash
cd /path/to/vr_control/boot_autostart_left_job

# 第一次只安装，不启用、不启动
sudo ./install.sh
nano ../left_job_orchestrator/env.systemd   # 改密码，并确认 OPENARM_ROS_SETUP
# 普通用户编译和验证全部维护链路
../migration_build/build_all.sh --clean
../migration_build/build_all.sh --check
# 启用开机启动（此刻仍不启动）
sudo ./install.sh --enable
# 清除之前启动失败的状态（不会启动服务）
sudo systemctl reset-failed openarm-left-job
# 现场安全后，可选立即启动
sudo systemctl start openarm-left-job
sudo systemctl status openarm-left-job
```

---

## 迁移到其他机器（一键）

**可以**，前提是目标机满足：

1. Linux + systemd
2. 整份 `vr_control` 仓库已放到目标机（任意路径，如 `/home/alice/vr_control`）
3. 有普通用户能跑真机（`install.sh` 用 `sudo` 时的 `SUDO_USER`，或仓库属主）
4. conda 环境 `openarm_teleop1` 存在，或在 `env.systemd` 里设 `OPENARM_PYTHON=`
5. 本机接 CAN，且能手动跑通 `./left_job_orchestrator/run_service.sh`

```bash
# 在目标机上
cd /任意路径/vr_control/boot_autostart_left_job
sudo ./install.sh
# 编辑 env 中的密码/token，并确认目标机 ROS setup 路径
nano ../left_job_orchestrator/env.systemd
../migration_build/build_all.sh --clean
sudo ./install.sh --enable
```

`install.sh` 会自动：

- 把单元里的 `/home/knowin/Desktop/vr_control` 换成**当前仓库根**
- 把 `User=` / `Group=` 换成当前安装用户
- 若不存在则从 example 生成 `env.systemd`
- 自动固定目标用户的 `openarm_teleop1` Python；非标准路径用 `--python`
- `--enable` / `--start` 前拒绝空密码或 `CHANGE_ME`，并检查 Python 和 ROS setup；失败时保持 disabled
- 在加载 ROS 环境后检查 vendor `dual_replay_control` 动态库（包括 `librcutils.so`）
- 服务启动前只读检查主树、vendor 和 KDL 构建结果
- 服务启动前等待 `follower_left` / `follower_right` 出现
- 只等待基础 `network.target`；TCP 可先监听，不会被未插线网卡的
  `network-online.target` 阻塞
- 启动失败不自动重试，避免重复执行 steady 动作

---

## 前提与限制

| 能自动处理 | 不能单靠脚本解决 |
| --- | --- |
| 路径、用户写入 systemd | CAN 硬件 / 驱动是否就绪 |
| 生成 env 文件 | conda 环境是否已装 |
| 安装/渲染 systemd | 轨迹、标定文件是否齐全 |
| | 现场安全（开机自动动臂） |

---

## 常用命令

```bash
# 查看 / 开启 / 关闭开机自启动
systemctl is-enabled openarm-left-job
sudo systemctl enable openarm-left-job
sudo systemctl disable openarm-left-job

# 当前会话启动 / 停止
sudo systemctl start openarm-left-job
sudo systemctl stop openarm-left-job
sudo systemctl restart openarm-left-job

# 同时切换开机自启动和当前运行状态
sudo systemctl enable --now openarm-left-job
sudo systemctl disable --now openarm-left-job

journalctl -u openarm-left-job -f
```

只编辑 `env.systemd` 不需要 `daemon-reload`，下次启动会重新读取。修改 service
模板、仓库路径或安装脚本后，应重新执行 `sudo ./install.sh --enable`，它会重新渲染
单元并执行 `daemon-reload`，但不会立即启动。开机自启动由 systemd 的
enabled/disabled 状态控制，不是 `env.systemd` 变量。

失败后安全恢复（最后一条会执行 steady，确认现场安全后才运行）：

```bash
nano ../left_job_orchestrator/env.systemd
sudo ./install.sh --enable
sudo systemctl reset-failed openarm-left-job
journalctl -u openarm-left-job -b -n 100 --no-pager
# 确认没有 VR/其它 CAN 控制链、急停可用、人员离开工作区后：
sudo systemctl start openarm-left-job
```
