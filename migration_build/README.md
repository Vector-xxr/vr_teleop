# 迁移机器编译包

`build_all.sh` 只做 CMake 配置、编译和静态验证，**不会配置 CAN、不会启动电机、不会让机器人运动**。

## 是否必须重新编译

换机器后建议重新编译。仓库中的旧二进制可能依赖原机器的：

- CPU / glibc / 动态库版本
- ROS KDL、urdfdom、yaml-cpp
- conda `openarm_teleop1` 中的 MuJoCo

各启动脚本虽然会在二进制缺失时临时编译一部分，但不应把“首次启动服务”当作迁移编译步骤。先统一编译、验证，再开真机更安全。

## 构建内容

| 构建树 | 输出 | 使用链路 |
| --- | --- | --- |
| `openarm_teleop/openarm_inplace/build/` | `quest_cartesian_control` | VR 遥控 / 录制 |
| 同上 | `dual_replay_control` | import_task、主树回放 |
| 同上 | `follower_record_control`、单双臂控制等 | 主从录制/回放辅助 |
| `.../skewer_topdown_2d_minimal/vendor/openarm_inplace/build_current/` | vendor `dual_replay_control` | 编排服务、`tcp_forward_grasp` |
| `.../vendor/openarm_vision_grasp_yolo/build/` | `kdl_fk_ik_tool` | 视觉抓取 FK/IK |

> 两个 `dual_replay_control` 不可混用：视觉抓取必须用 vendor 版本，因为它包含 `tcp_vel` approach stream。

## 目标机前提

1. Linux（推荐与当前机相同发行版）
2. CMake ≥ 3.22、C++17 编译器
3. `openarm_teleop1` conda 环境，且其中可 `import mujoco`
4. KDL / kdl_parser / urdfdom / Eigen / yaml-cpp / GLFW/OpenGL 开发包
5. 整份仓库完整，尤其 vendor 源码与 `recordings/`

已有 Ubuntu/ROS 环境可直接试编译。缺系统包时，先看帮助，再明确执行：

```bash
cd /path/to/vr_control
./migration_build/build_all.sh --help
./migration_build/build_all.sh --install-deps
```

脚本会只在 `apt-get` 阶段调用 `sudo`，编译本身仍以普通用户运行，避免生成 root 所有者的构建文件。

如果 `libkdl-parser-dev` 不存在，先按项目环境文档安装 ROS Humble：

```text
openarm_teleop/openarm_inplace/environment/openarm_teleop1.system_deps.md
```

## 推荐迁移顺序

```bash
cd /path/to/vr_control

# 1. 选择正确 Python（默认会找 ~/miniconda3/envs/openarm_teleop1）
export OPENARM_PYTHON="$HOME/miniconda3/envs/openarm_teleop1/bin/python"

# 2. 全量重建关键链路
./migration_build/build_all.sh --clean

# 3. 再做一次只读检查
./migration_build/build_all.sh --check

# 4. 安装 systemd，但先不启动（不会动臂）
sudo ./boot_autostart_left_job/install.sh --no-start

# 5. 改目标机密码/token
nano ./left_job_orchestrator/env.systemd

# 6. 确认 CAN、急停、现场安全后才启动
sudo systemctl start openarm-left-job
```

## 常用参数

```bash
./migration_build/build_all.sh                 # 增量编译
./migration_build/build_all.sh --clean         # 删除三个生成目录后重建
./migration_build/build_all.sh --jobs 4        # 限制并行数
./migration_build/build_all.sh --check         # 只检查，不编译、不碰硬件
```

## 说明

- 此脚本覆盖当前维护的 VR、回放、编排、视觉抓取链路，不编译无关的 ROS2 工作空间、KER 数据采集或历史顶层副本。
- 编译成功不等于 CAN/相机/网络已配置；真机前仍需逐项检查硬件。
- 不要把 `OPENARM_DUAL_BUILD_DIR` 指向主树来跑视觉抓取。
