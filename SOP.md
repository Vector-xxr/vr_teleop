# OpenArm 启动与部署 SOP

工程根目录：

```bash
cd /home/knowin/Desktop/vr_control
```

> 同一时间只能运行一条真机控制链路。VR 遥控和编排服务器都会占用
> `follower_left` / `follower_right`，禁止同时启动。

## 1. 真机 VR 遥控

### 正常启动

```bash
cd /home/knowin/Desktop/vr_control
export OPENARM_SUDO_PASSWORD=你的sudo密码

OPENARM_QUEST_EXTRA_ARGS="--teleop-mode human_mimic" \
OPENARM_HM_IK_TCP=1 \
OPENARM_HM_PANEL_BYPASS_SOFT=1 \
./quest/run_openarm_cartesian_bimanual.sh
```

正常启动会先执行：

```text
openarm_teleop/openarm_inplace/recordings/tasks/steady_action_on_table.csv
```

随后进入 VR 遥控。Quest 浏览器打开终端打印的：

```text
https://<电脑局域网IP>:8443/client.html
```

- Grip 按住：跟随手柄；松开：冻结。
- 右手 A 闭合、B 张开；左手 X 闭合、Y 张开。
- 停止：控制终端按 `Ctrl+C`，或在录制面板选择“退出”。

### 跳过启动准备动作

```bash
cd /home/knowin/Desktop/vr_control
export OPENARM_SUDO_PASSWORD=你的sudo密码

OPENARM_QUEST_STARTUP_MOVE=0 \
OPENARM_QUEST_EXTRA_ARGS="--teleop-mode human_mimic" \
OPENARM_HM_IK_TCP=1 \
OPENARM_HM_PANEL_BYPASS_SOFT=1 \
./quest/run_openarm_cartesian_bimanual.sh
```

### 单独启动摄像头 FPV bridge

```bash
cd /home/knowin/Desktop/vr_control
./quest/run_quest_camera_bridge.sh
```

## 2. 编排服务器

### 手动启动服务器

```bash
cd /home/knowin/Desktop/vr_control/left_job_orchestrator
export OPENARM_SUDO_PASSWORD=你的sudo密码
./run_service.sh
```

启动成功应看到：

```text
ready. TCP :47925
```

服务会先执行一次 steady，再等待任务命令。

### 测试服务器

另开终端：

```bash
cd /home/knowin/Desktop/vr_control
export PY=/home/knowin/miniconda3/envs/openarm_teleop1/bin/python
HOST=127.0.0.1  # 跨机器时改成机器人局域网 IP

$PY -m left_job_orchestrator.test_socket --host "$HOST" status
$PY -m left_job_orchestrator.test_socket --host "$HOST" run --job 1
$PY -m left_job_orchestrator.test_socket --host "$HOST" interactive
```

任务文件放在：

```text
openarm_teleop/openarm_inplace/recordings/jobN/
```

至少需要 `action1.csv`。动作和视觉抓取按末尾数字执行；同编号时视觉先执行：

```text
action1.csv → visual_grasp2.txt → action2.csv → action3.csv
```

## 3. 自动编译

脚本位置：

```text
migration_build/build_all.sh
```

### 首次迁移：安装依赖并完整编译

```bash
cd /home/knowin/Desktop/vr_control
export OPENARM_PYTHON=/home/knowin/miniconda3/envs/openarm_teleop1/bin/python

./migration_build/build_all.sh --install-deps --clean
./migration_build/build_all.sh --check
```

### 日常完整重新编译

```bash
cd /home/knowin/Desktop/vr_control
export OPENARM_PYTHON=/home/knowin/miniconda3/envs/openarm_teleop1/bin/python

./migration_build/build_all.sh --clean
./migration_build/build_all.sh --check
```

### 仅检查现有编译结果

```bash
cd /home/knowin/Desktop/vr_control
./migration_build/build_all.sh --check
```

编译必须由普通用户执行，不要使用 `sudo` 运行整个编译脚本；脚本仅在安装系统依赖时调用 sudo。

## 4. 编排服务器开机自启动

安装包：

```text
boot_autostart_left_job/
```

systemd 服务名：

```text
openarm-left-job
```

### 首次安装

```bash
cd /home/knowin/Desktop/vr_control/boot_autostart_left_job

# 安装服务文件，但不启用、不启动
sudo ./install.sh

# 必须改密码；并确认 OPENARM_ROS_SETUP=/opt/ros/humble/setup.bash 存在
nano ../left_job_orchestrator/env.systemd
# nano 保存：Ctrl+O，回车确认；退出：Ctrl+X

# 确认编译结果完整
../migration_build/build_all.sh --check

# 开启开机自启动，但当前不启动
sudo ./install.sh --enable

# 若之前启动失败，清除 failed 状态（不会启动、不会动臂）
sudo systemctl reset-failed openarm-left-job

# 确认 VR/其它 CAN 控制链已停、急停可用、工作区无人后启动
sudo systemctl start openarm-left-job
```

也可以在配置完成后直接执行：

```bash
sudo ./install.sh --enable  # 安装并开启开机自启动，不立即启动
sudo ./install.sh --start   # 安装、开启开机自启动并立即启动
```

`--enable` / `--start` 会拒绝空密码或 `CHANGE_ME`，确认 Python、ROS setup
可读，并在加载 ROS 后检查 vendor `dual_replay_control` 的动态库。检查不会访问
硬件；`--start` 和 `systemctl start/restart` 会执行 steady，可能立即动臂。

只编辑 `left_job_orchestrator/env.systemd` 不需要 `daemon-reload` 或重新安装；
服务下次启动会读取新值。修改 service 模板、仓库路径或安装脚本后，重新执行
`sudo ./install.sh --enable`（会自动 `daemon-reload`，不会立即启动）。

### 查询状态

```bash
systemctl is-enabled openarm-left-job
systemctl is-active openarm-left-job
sudo systemctl status openarm-left-job
```

### 当前最常用命令

```bash
# 开启开机自启动，但现在不启动
sudo systemctl enable openarm-left-job

# 关闭开机自启动，但不停止当前服务
sudo systemctl disable openarm-left-job

# 启动 / 停止当前服务
sudo systemctl start openarm-left-job
sudo systemctl stop openarm-left-job

# 立即停止并关闭开机自启动
sudo systemctl disable --now openarm-left-job

# 查看是否开机启动、当前是否运行
systemctl is-enabled openarm-left-job
systemctl is-active openarm-left-job
```

### 开启或关闭开机自启动

```bash
# 开启开机自启动，不立即启动
sudo systemctl enable openarm-left-job

# 关闭开机自启动，不停止当前运行的服务
sudo systemctl disable openarm-left-job
```

### 启动或停止当前服务

```bash
# 立即启动；不改变下次开机是否启动
sudo systemctl start openarm-left-job

# 立即停止；如果仍为 enabled，下次开机还会启动
sudo systemctl stop openarm-left-job

# 重新启动
sudo systemctl restart openarm-left-job
```

### 同时切换开机自启动和当前运行状态

```bash
# 开启开机自启动并立即启动
sudo systemctl enable --now openarm-left-job

# 立即停止并关闭以后开机自启动
sudo systemctl disable --now openarm-left-job
```

### 查看服务日志

```bash
journalctl -u openarm-left-job -f
```

### 重启公共机并检查自启动

```bash
sudo reboot

# 机器重新登录后检查
systemctl is-enabled openarm-left-job
systemctl is-active openarm-left-job
sudo systemctl status openarm-left-job
journalctl -u openarm-left-job -b -n 100 --no-pager
```

开机失败后的安全恢复：

```bash
cd /home/knowin/Desktop/vr_control/boot_autostart_left_job
nano ../left_job_orchestrator/env.systemd
sudo ./install.sh --enable
sudo systemctl reset-failed openarm-left-job
journalctl -u openarm-left-job -b -n 100 --no-pager

# 仅在现场安全检查完成后执行：
sudo systemctl start openarm-left-job
```

若日志出现 `librcutils.so: cannot open shared object file`，确认
`OPENARM_ROS_SETUP=/opt/ros/humble/setup.bash`（或目标机实际 ROS 路径），再执行
上面的 `install.sh --enable` 只读检查。不要把交互 shell 中的 `source` 当作
systemd 环境；服务会由 `run_service.sh` 自行加载 ROS。

若刚开机时状态显示 `start waiting` 且
`systemd-networkd-wait-online.service` 正在等待未插网线的网卡，重新运行最新
`sudo ./install.sh --enable`。最新 unit 只依赖 `network.target`，不会因无关网卡
阻塞约两分钟。

### 彻底卸载

```bash
cd /home/knowin/Desktop/vr_control/boot_autostart_left_job
sudo ./uninstall.sh
```
