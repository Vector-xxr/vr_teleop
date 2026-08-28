# 公共机部署 KER 轻量遥操 / 录制 / 回放

给 Cursor（或人）在**另一台公共机**上部署用。目标：只依赖 `KER/` + 少量外部包，不依赖 Quest/`openarm_inplace` 回放面板。

---

## 给 Cursor 的部署提示词（可直接粘贴）

```text
请按仓库 KER/DEPLOY_PUBLIC.md 在本机部署 KER 轻量遥操栈。
范围：真机遥操 + 录制面板 + 200Hz 回放（不要装 dora 全栈，除非我另说）。
步骤：检查/创建 conda 环境 openarm_teleop1 → pip install -r KER/requirements.txt
→ 安装 openarm_can（见文档）→ 配置 CAN can0/can1 与 KER USB → 冒烟验证。
硬编码路径用 KER/env.sh；不要提交 sudo 密码到 git。
完成后给出：能跑的命令、本机实际路径、未解决项。
```

---

## 1. 要拷什么

### 最小集（真机遥操 + 录制 + 回放）

从开发机拷到公共机同一相对布局，例如 `~/openarm_teleop1/KER/`：

| 路径 | 说明 |
|------|------|
| `KER/*.py` | 遥操/录制/回放/映射 |
| `KER/*.sh` + `KER/env.sh` | 启动脚本 |
| `KER/vendor/openarm_ker/` | KER USB 协议（已 vendor） |
| `KER/requirements.txt` | pip 依赖 |
| `KER/README_RUN.md` | 日常用法 |
| `KER/DEPLOY_PUBLIC.md` | 本文 |
| `KER/.deps/openarm_can/`（若有）或旁路 `openarm_teleop/openarm_can/` | CAN Python 绑定源码 |

**不要依赖**：`quest/`、`openarm_inplace` 回放面板、Quest 1kHz CSV 回放。

### 可选

| 路径 | 说明 |
|------|------|
| `openarm_teleop/openarm_mujoco/v1/scene_table.xml`（及资源） | 仅 MuJoCo 仿真 |
| `KER/dora-openarm-data-collection/` | 旧 dora 采集，**默认可不拷**（体积大） |
| `KER/recordings/` | 已有动作；回放才需要 |

### 建议 rsync（开发机上）

```bash
# 最小（不含 dora、不含 recordings）
rsync -a --exclude 'dora-openarm-data-collection' --exclude 'recordings' --exclude '__pycache__' \
  /home/lan/openarm_teleop1/KER/  user@public:~/openarm_teleop1/KER/

# openarm_can 源码（若公共机 KER/.deps 没有）
rsync -a /home/lan/openarm_teleop1/openarm_teleop/openarm_can/ \
  user@public:~/openarm_teleop1/openarm_teleop/openarm_can/
```

---

## 2. 系统对齐清单（公共机必须核对）

| 项 | 开发机（参考） | 公共机要对齐 |
|----|----------------|--------------|
| 右臂 CAN | `can0` | `OPENARM_RIGHT_CAN`（默认 can0） |
| 左臂 CAN | `can1` | `OPENARM_LEFT_CAN`（默认 can1） |
| CAN-FD | 1M / 5M | `OPENARM_CAN_BITRATE` / `OPENARM_CAN_DBITRATE` |
| KER USB | `303a:4002` | `lsusb -d 303a:4002`，udev 或 chmod |
| 关节映射 | swap_j67、negate_j7、invert_left_grip | 已写在 `run_ker_real_teleop.sh` |
| Python | conda `openarm_teleop1` | `OPENARM_PYTHON` 或同名 env |
| 录制口 | UDP `127.0.0.1:47950` | 一般不动 |
| 回放 | `./run_ker_replay.sh` ~200Hz | **不要**用 Quest dual_replay |
| sudo | CAN/USB 配置需要 | `OPENARM_SUDO_PASSWORD`（仅本机 env，勿提交） |

若公共机左右 CAN 对调，只改环境变量即可：

```bash
export OPENARM_RIGHT_CAN=can1
export OPENARM_LEFT_CAN=can0
```

本机（`vr_control` 公共机）若已有 udev 别名，`env.sh` 会自动选用：

- `OPENARM_RIGHT_CAN=follower_right`
- `OPENARM_LEFT_CAN=follower_left`

（没有 `follower_*` 时仍回退 `can0`/`can1`。）

硬件一键对齐（CAN-FD + KER USB chmod）：

```bash
cd /home/knowin/Desktop/vr_control/KER
export OPENARM_SUDO_PASSWORD='...'   # 勿提交到 git
./align_hw_once.sh
```

---

## 3. 系统依赖安装

### 3.1 系统包（Ubuntu 示例）

```bash
sudo apt update
sudo apt install -y \
  can-utils iproute2 \
  build-essential cmake git \
  libusb-1.0-0 \
  python3-tk \
  libcli11-dev
# openarm_can 若需系统库，按该仓库 README 再装 libopenarm-can / 源码编译
```

### 3.2 Conda 环境

```bash
# 已有则跳过
conda create -n openarm_teleop1 python=3.11 -y
conda activate openarm_teleop1
cd ~/openarm_teleop1/KER   # 按实际路径
pip install -r requirements.txt
```

### 3.3 安装 `openarm_can`

优先顺序：

1. 若 `KER/.deps/openarm_can/python` 存在 → 对该目录 `pip install -e .`
2. 否则对 `../openarm_teleop/openarm_can/python` 同样安装
3. 构建失败时：按 [openarm_can](https://github.com/enactic/openarm_can) 文档先装 C++ 库再装 Python 绑定

```bash
source ~/openarm_teleop1/KER/env.sh
echo "OPENARM_CAN_PYTHON=$OPENARM_CAN_PYTHON"
cd "$OPENARM_CAN_PYTHON"
pip install -e .
python -c "import openarm_can; print('ok', openarm_can)"
```

### 3.4 KER USB 权限（可选 udev）

```bash
# 临时
BUS=$(lsusb -d 303a:4002 | awk '{print $2}')
DEV=$(lsusb -d 303a:4002 | awk '{print $4}' | tr -d ':')
sudo chmod a+rw /dev/bus/usb/${BUS}/${DEV}

# 或 udev（示例）：内核看到 303a:4002 后给 plugdev
# ATTR{idVendor}=="303a", ATTR{idProduct}=="4002", MODE="0666"
```

### 3.5 CAN-FD

```bash
# 脚本 run_ker_real_teleop.sh 会配置；也可手动：
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
# can1 同理
ip -br link | grep can
```

---

## 4. 冒烟验证（按顺序）

```bash
cd ~/openarm_teleop1/KER
source ./env.sh
echo "PY=$OPENARM_PYTHON"

# 1) 模块
"$OPENARM_PYTHON" -c "import numpy,usb,openarm_can; print('imports ok')"

# 2) KER USB
lsusb -d 303a:4002

# 3) 真机遥操（机械臂周围清空）
./run_ker_real_teleop.sh
# 期望日志：mapping ... negate_j7=True invert_left_grip=True
#          soft start: hold ... → chase ... → tracking
# 面板：开始/停止录制 → recordings/sessions/...

# 4) 回放（停掉遥操后）
pkill -f ker_real_teleop.py || true
./run_ker_replay.sh recordings/sessions/<session>/<seg>
# 或旧文件：./run_ker_replay.sh recordings/sessions/.../1.csv

# 5) 可选仿真（需 scene_table.xml）
./run_ker_mujoco_sim.sh --dummy
```

---

## 5. 目录与入口（集成现状）

全部日常入口已在 `KER/`：

| 功能 | 入口 |
|------|------|
| 真机遥操+录制 | `./run_ker_real_teleop.sh` |
| 仅录制面板 | `./run_ker_record_panel.sh` |
| 200Hz 回放 | `./run_ker_replay.sh` |
| MuJoCo | `./run_ker_mujoco_sim.sh` |
| 映射 | `ker_joint_map.py` |
| KER USB | `vendor/openarm_ker/` |
| 环境解析 | `env.sh` |

**仍在 KER 外的**：`openarm_can` 源码/编译产物、MuJoCo `scene_table.xml` 资源、conda 环境本身。用 `env.sh` 自动查找旁路路径。

---

## 6. 映射与行为（勿在公共机随意改）

与当前开发机一致（`run_ker_real_teleop.sh` 已写死）：

- `--swap-j67`
- `--no-negate-j6`
- `--negate-j7`
- `--invert-left-grip`
- soft start：hold ≈0.8s → chase ≈4s（满增益限速命令）→ free track

录制：`recordings/sessions/<名>/<段>/real_teleop_dual_merged.csv`  
原则：**录=控**，~200Hz，**禁止插值上采样到 1kHz**。

---

## 7. 常见问题

| 现象 | 处理 |
|------|------|
| `libyaml-cpp.so.0.8`（若误用 Quest 回放） | KER 回放不走该二进制；用 `run_ker_replay.sh` |
| `SocketCAN follower_right` | KER 用 `can0/can1`，不是 udev 别名 |
| USB `Resource busy` / Access denied | `pkill -f ker_real_teleop`；chmod/udev |
| `openarm_can` import 失败 | 检查 `pip show openarm_can` 与 `OPENARM_CAN_PYTHON` |
| 启动突变 / 跟不紧 | 确认已是新版 SlowChase（满增益 + 定时进 track） |
| 回放卡顿 | 确认用的是 `run_ker_replay.sh` 而非 Quest panel |

---

## 8. 公共机 Cursor 完成后应回报

1. `OPENARM_PYTHON` / `OPENARM_CAN_PYTHON` / CAN 口实际值  
2. 冒烟命令与是否通过  
3. 与开发机不一致的硬件差异（CAN 对调、USB 权限方式等）  
4. 未完成项（例如未装 MuJoCo 场景）
