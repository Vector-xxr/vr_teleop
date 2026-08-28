# openarm_teleop1 系统依赖说明

本文件补充 `openarm_teleop1.environment.yml` 没有覆盖的系统依赖。

## ROS2

当前环境里能看到大量 ROS2 Python 包，例如 `rclpy`、`sensor-msgs`、`trajectory-msgs`、`xacro` 等。它们通常来自系统 ROS2 Humble，而不是 conda 环境本身。

建议安装并使用：

```bash
sudo apt install ros-humble-desktop ros-humble-xacro ros-humble-ros2-control ros-humble-ros2-controllers
source /opt/ros/humble/setup.bash
```

## C++ / OpenArm 真机链路

`openarm_inplace` 的 C++ teleop/replay 需要：

```bash
sudo apt install build-essential cmake pkg-config can-utils
```

真机 CAN 使用前需要配置 SocketCAN，例如项目中的：

```bash
cd <工程根>/openarm_teleop/openarm_can   # 或现场实际 openarm_can 路径
sudo setup/openarm-can-configure-socketcan-4-arms
```

## 创建环境

```bash
cd <工程根>/openarm_teleop/openarm_inplace
conda env create -f environment/openarm_teleop1.environment.yml
conda activate openarm_teleop1
```

工程根也可：`conda env create -f environment/openarm_teleop1.environment.yml`（见仓库根 `ENV_SETUP.md`）。

如果环境已存在，更新：

```bash
conda env update -n openarm_teleop1 -f environment/openarm_teleop1.environment.yml --prune
```
