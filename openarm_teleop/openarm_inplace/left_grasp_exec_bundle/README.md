# skewer_topdown_2d_minimal

这是当前工作区中的最小可运行目录：

```text
/home/wenjiajie/code/drink/skewer_topdown_2d_minimal
```

目录内脚本优先使用本目录下的 `vendor/` 和 `outputs/`，便于后续和遥操流程结合。

## 包内包含什么

```text
skewer_topdown_2d_minimal/
  README.md
  README_FULL.md
  SOP_FULL.md
  requirements.txt
  config/
    skewer_align_config.json
  run_skewer_2d_servo.py
  scripts/
    align_once_move.py
    auto_calibrate_pixel_to_motion.py
    pick_target_center.py
    record_right_observe_with_camera.py
    check_minimal_package.py
  vendor/
    openarm_inplace/
      script/
      control/
      src/
    openarm_can/
    openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf
    openarm_vision_grasp_yolo/
      scripts/solve_openarm_fk_ik.py
      cpp/
      robot/
    template_library/rgbd_samurai_minimal/01_capture_rgbd.py
  outputs/
    right_observe_record_20260723_135046/
      real_follower_right_03.csv
      follower_record.log
    right_observe_record_20260723_192233/
      follower_position_snapshots.txt
      follower_record.log
```

没有复制历史 `outputs/align_once`、`outputs/auto_xy_calib`、离线检测图片、缓存文件和大批调试结果。

## 已内置的工程依赖

代码会优先使用包内 `vendor/`：

```text
vendor/openarm_inplace
  真机 replay、CAN follower 控制、录制面板。

vendor/openarm_can
  openarm_inplace 的 SocketCAN 电机通信源码。

vendor/openarm_description/.../v1.urdf
  FK/IK 和 C++ 控制使用的 OpenArm URDF。

vendor/openarm_vision_grasp_yolo/scripts/solve_openarm_fk_ik.py
  OpenArm 7 关节 FK/IK 求解。

vendor/template_library/rgbd_samurai_minimal/01_capture_rgbd.py
  Orbbec 相机读取。只在 --camera-backend orbbec 时需要。
```

如果 `vendor/` 被删除，脚本才会回退查找本目录父级下的旧同级工程目录：

```text
../openarm_inplace
../openarm_vision_grasp_yolo
../template_library
```

仍然需要系统环境本身具备：`conda anygrasp`、OpenCV/numpy、SocketCAN、sudo/ip、ROS Humble KDL/urdfdom/yaml-cpp 等 C++ 运行库。当前目录保留源码和脚本；`vendor/openarm_inplace/build/*`、`vendor/openarm_vision_grasp_yolo/build/*` 这类编译产物不保证随文档存在。如果真机 replay 或 FK/IK 工具缺失，请在对应 `vendor/` 子目录里重新 `cmake` 编译。

## Python 依赖

推荐直接使用当前环境：

```bash
conda activate anygrasp
```

最小 Python 包：

```text
numpy
opencv-python
pyorbbecsdk  # 仅 Orbbec 后端需要
```

## CAN 对应关系

当前现场约定：

```text
can0 = 右臂
can1 = 左臂
```

本包默认控制右臂：

```text
--follower-can can0
```

## 自检

```bash
cd /home/wenjiajie/code/drink/skewer_topdown_2d_minimal
conda activate anygrasp

python scripts/check_minimal_package.py
python -m py_compile run_skewer_2d_servo.py scripts/*.py
```

`check_minimal_package.py` 不会打开相机，也不会动机械臂，只检查文件、vendor 运行件和 Python 模块。

## 离线检测

先用保存下来的 RGB 测检测：

```bash
python run_skewer_2d_servo.py \
  --stage detect-image \
  --image /path/to/latest_rgb.png \
  --output-dir outputs/offline_check \
  --camera-backend opencv \
  --black-v-max 80 \
  --min-area-px 250 \
  --target-selection red \
  --target-center-px=348.0,364.0
```

当前代码支持：

```text
优先检测黑色细长段
黑色段检测失败时，自动 fallback 检测红色套/红色标记
```

如果红色套较暗：

```bash
--red-v-min 35
```

如果误检手或背景红色：

```bash
--red-s-min 150
```

## 手动快抓

按 `d` 抓一根，适合调试。

## 遥操动作串接抓取

流程：

```text
播放遥操动作 a -> 平滑插值到观测位 -> 自动检测并抓取 -> 垂直抬起 -> 平滑插值到遥操动作 c 第一帧 -> 播放遥操动作 c
```

只需要把下面两个路径替换成你录制的动作：

```text
--pre-replay   遥操动作 a
--post-replay  遥操动作 c
```

执行命令：

```bash
cd /home/wenjiajie/code/drink/skewer_topdown_2d_minimal
conda activate anygrasp

python scripts/align_once_move.py \
  --pre-replay outputs/PATH_TO_ACTION_A.csv \
  --observe-replay outputs/right_observe_record_20260727_154012/real_follower_right_02.csv \
  --post-replay outputs/PATH_TO_ACTION_C.csv \
  --initial-transition-duration 0.0 \
  --pre-to-observe-duration 3.0 \
  --post-replay-transition-duration 3.0 \
  --quick-grasp-on-d \
  --auto-once \
  --after-grasp-action lift-only \
  --auto-start-delay-s 1.0 \
  --auto-detect-retry-s 1.0 \
  --no-quick-align-angle \
  --max-move-mm 45 \
  --xy-correction-scale 0.95 \
  --grasp-descent-m 0.045 \
  --grasp-descent-duration 5.5 \
  --pre-close-hold-duration 0.6 \
  --gripper-open -0.40 \
  --gripper-close -0.0 \
  --gripper-close-duration 1.2 \
  --gripper-close-hold-duration 1.2 \
  --post-grasp-lift-m 0.04 \
  --min-post-grasp-lift-m 0.02 \
  --post-grasp-lift-duration 3.0 \
  --execute \
  --follower-can can0 \
  --follower-only \
  --sudo-password 888012 \
  --camera-backend opencv \
  --camera-index 8 \
  --target-selection red \
  --red-h-high 145 \
  --red-v-min 35 \
  --red-s-min 80 \
  --red-min-area-px 80 \
  --red-max-area-px 4000 \
  --red-roi-left 0.25 \
  --red-roi-right 0.75 \
  --red-roi-top 0.05 \
  --red-roi-bottom 0.45 \
  --red-max-target-dist-px 260 \
  --track-roi-px 110 \
  --no-track-fallback-to-full
```

```bash
python scripts/align_once_move.py \
  --observe-replay outputs/right_observe_record_20260723_135046/real_follower_right_03.csv \
  --quick-grasp-on-d \
  --no-quick-align-angle \
  --max-move-mm 180 \
  --xy-correction-scale 1 \
  --grasp-descent-m 0.04 \
  --grasp-descent-duration 5.0 \
  --pre-close-hold-duration 0.6 \
  --gripper-close-duration 1.2 \
  --gripper-close-hold-duration 1.2 \
  --post-grasp-lift-m 0.10 \
  --post-grasp-lift-duration 3.0 \
  --place-snapshot outputs/right_observe_record_20260723_192233/follower_position_snapshots.txt \
  --place-duration 3.0 \
  --gripper-open -0.5 \
  --gripper-close 0.1 \
  --quick-detect-preview-s 0.25 \
  --execute \
  --follower-can can0 \
  --follower-only \
  --sudo-password 888012 \
  --camera-backend opencv \
  --camera-index 8 \
  --black-v-max 80 \
  --min-area-px 250 \
  --target-selection red \
  --track-roi-px 90 \
  --no-track-fallback-to-full
```

## 自动循环抓取

无需按键，回到观测位后自动抓下一根。

```bash
python scripts/align_once_move.py \
  --observe-replay outputs/right_observe_record_20260723_135046/real_follower_right_03.csv \
  --quick-grasp-on-d \
  --auto-cycle \
  --auto-start-delay-s 1.0 \
  --auto-cycle-delay-s 1.0 \
  --auto-detect-retry-s 1.0 \
  --no-quick-align-angle \
  --max-move-mm 180 \
  --xy-correction-scale 1 \
  --grasp-descent-m 0.04 \
  --grasp-descent-duration 5.0 \
  --pre-close-hold-duration 0.6 \
  --gripper-close-duration 1.2 \
  --gripper-close-hold-duration 1.2 \
  --post-grasp-lift-m 0.10 \
  --post-grasp-lift-duration 3.0 \
  --place-snapshot outputs/right_observe_record_20260723_192233/follower_position_snapshots.txt \
  --place-duration 3.0 \
  --gripper-open -0.5 \
  --gripper-close 0.1 \
  --quick-detect-preview-s 0.25 \
  --execute \
  --follower-can can0 \
  --follower-only \
  --sudo-password 888012 \
  --camera-backend opencv \
  --camera-index 8 \
  --black-v-max 80 \
  --min-area-px 250 \
  --target-selection red \
  --track-roi-px 90 \
  --no-track-fallback-to-full
```

## 单次自动到放置位，不松爪

抓完只到放置位，夹爪保持闭合，方便后面接遥操。

```bash
python scripts/align_once_move.py \
  --observe-replay outputs/right_observe_record_20260723_135046/real_follower_right_03.csv \
  --quick-grasp-on-d \
  --auto-once \
  --auto-start-delay-s 1.0 \
  --auto-detect-retry-s 1.0 \
  --no-quick-align-angle \
  --max-move-mm 180 \
  --xy-correction-scale 1 \
  --grasp-descent-m 0.04 \
  --grasp-descent-duration 5.0 \
  --pre-close-hold-duration 0.6 \
  --gripper-close-duration 1.2 \
  --gripper-close-hold-duration 1.2 \
  --post-grasp-lift-m 0.10 \
  --post-grasp-lift-duration 3.0 \
  --place-snapshot outputs/right_observe_record_20260723_192233/follower_position_snapshots.txt \
  --place-duration 3.0 \
  --gripper-open -0.5 \
  --gripper-close 0.1 \
  --quick-detect-preview-s 0.25 \
  --execute \
  --follower-can can0 \
  --follower-only \
  --sudo-password 888012 \
  --camera-backend opencv \
  --camera-index 8 \
  --black-v-max 80 \
  --min-area-px 250 \
  --target-selection red \
  --track-roi-px 90 \
  --no-track-fallback-to-full
```

## 安全提醒

```text
不要同时运行多个占用 can0 的 replay、遥操或 record 进程。
第一次测试新命令时，先用手动快抓，不要直接自动循环。
检测框落错目标时，自动模式下立即 q/Esc 退出。
```
