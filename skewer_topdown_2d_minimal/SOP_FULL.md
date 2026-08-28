# 烧烤签 2D 抓取 SOP

本文档用于现场操作。默认使用右臂、右腕相机、`can1`。

## 0. 环境准备

```bash
cd /home/wenjiajie/code/openarm_yaocao/openarm_teleop-openarm_teleop/vr_teleop-teleop-full/skewer_topdown_2d
conda activate anygrasp
```

确认：

```text
右臂 follower CAN = can1
右腕相机 OpenCV index = 8
观测动作 = outputs/right_observe_record_20260723_135046/real_follower_right_03.csv
配置文件 = config/skewer_align_config.json
```

机械臂周围清空，手放急停附近。不要同时运行其他占用 `can1` 的遥操或回放进程。

## 命令模式怎么选

现场抓取主要有三种命令：

```text
手动快抓
  命令只带 --quick-grasp-on-d。
  程序到观测位后等待按键，按 d 抓一根。
  抓完去放置位松开，再回观测位闭合。
  用途：调试、第一次测试、想人工确认每一步。

自动循环
  命令带 --quick-grasp-on-d 和 --auto-cycle。
  到观测位后等待 1s 自动抓；抓完去放置位松开，回观测位闭合；再等待 1s 抓下一根。
  用途：连续抓多根。

单次自动到放置位不松爪
  命令带 --quick-grasp-on-d 和 --auto-once。
  到观测位后等待 1s 自动抓；抓完去放置位，但夹爪保持闭合。
  用途：只抓一根，后面要接遥操动作。
```

不要同时传 `--auto-cycle` 和 `--auto-once`。这两个模式互斥。

## 1. 离线检测

目的：先确认黑色涂层段能被稳定检测到，再上机械臂。

先准备一张当前场景 RGB 图片，例如：

```text
outputs/align_once/align_once_YYYYMMDD_HHMMSS/detections/XXX/latest_rgb.png
```

运行离线检测：

```bash
python run_skewer_2d_servo.py \
  --stage detect-image \
  --image outputs/align_once/align_once_YYYYMMDD_HHMMSS/detections/XXX/latest_rgb.png \
  --output-dir outputs/skewer_2d_servo_test \
  --black-v-max 80 \
  --min-area-px 250 \
  --target-selection skewer \
  --target-center-px=348.0,364.0
```

检查输出：

```text
outputs/skewer_2d_servo_test/detect_image_*/latest_vis.png
outputs/skewer_2d_servo_test/detect_image_*/latest_detection.json
```

合格标准：

```text
latest_vis.png 中黄/红色标记落在黑色涂层抓取段上，或落在红色套/红色标记中心
检测框没有落在夹爪、阴影、支架或背景文字上
latest_detection.json 中 detected=true
```

如果检测不到：

```text
黑色太浅：增大 --black-v-max，例如 100
误检杂物：增大 --min-area-px，例如 300
目标太小：减小 --min-area-px，例如 150
当前带红色套的签子：保持默认 --red-fallback；必要时降低 --red-v-min，例如 35
红色误检皮肤/背景：提高 --red-s-min，例如 100
```

## 2. 运行偏移标定脚本

目的：计算“图像像素误差”到“机械臂 base XY 平移”的映射。只有当相机、观测位、夹爪中心像素变了，或者长期对不准时才需要重新标定。

启动标定：

```bash
python scripts/auto_calibrate_pixel_to_motion.py \
  --observe-replay outputs/right_observe_record_20260723_135046/real_follower_right_03.csv \
  --step-mm 10 \
  --max-offset-mm 60 \
  --execute \
  --update-align-config \
  --follower-can can1 \
  --follower-only \
  --sudo-password 888012 \
  --camera-backend opencv \
  --camera-index 8 \
  --black-v-max 80 \
  --min-area-px 250 \
  --target-selection skewer \
  --target-center-px=348.0,364.0
```

标定窗口按键：

```text
w  base +X 移动 10mm
s  base -X 移动 10mm
d  base +Y 移动 10mm
a  base -Y 移动 10mm
c  回观测中心
1  保存当前画面排序第 1 根签
2  保存当前画面排序第 2 根签
3  保存当前画面排序第 3 根签
f  拟合当前数据
q  退出并最终拟合
```

推荐采集方式：

```text
每移动到一个位置后，按 1 或 2 或 3 保存同一根目标。
标定一套矩阵时，最好跟踪同一根签在不同机器人偏移下的位置变化。
不要把不同签子随意混成同一个标定目标，否则 RMS 会很大，矩阵会乱。
```

推荐覆盖：

```text
中心
X 正方向
X 负方向
Y 正方向
Y 负方向
四个对角区域
尽量覆盖实际抓取会出现的画面范围
```

质量判断：

```text
二次 RMS 小于 2mm：很好
二次 RMS 2 到 4mm：可以测试
二次 RMS 大于 4mm：不要更新配置，需要重新采
```

脚本已经有质量门，坏结果不会轻易覆盖：

```text
config/skewer_align_config.json
```

## 3. 真机抓取模式一：手动快抓

目的：先让机器人到观测位，打开相机，然后人工按 `d` 触发一轮抓取。这个模式最适合调试，因为你可以先看画面和检测结果。

```bash
cd /home/wenjiajie/code/openarm_yaocao/openarm_teleop-openarm_teleop/vr_teleop-teleop-full/skewer_topdown_2d
conda activate anygrasp

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
  --follower-can can1 \
  --follower-only \
  --sudo-password 888012 \
  --camera-backend opencv \
  --camera-index 8 \
  --black-v-max 80 \
  --min-area-px 250 \
  --target-selection skewer \
  --track-roi-px 90 \
  --no-track-fallback-to-full
```

按键：

```text
d  检测并抓一根
r  清除目标锁定
s  保存当前 RGB
q  退出
Esc 退出
```

按 `d` 后完整流程：

```text
检测黑色涂层段并显示 0.25s
XY 平移到夹爪中心
开爪
下探 4cm，用时 5.0s
最低点保持 0.6s
闭爪 1.2s
闭爪后保持 1.2s
上升 10cm，用时 3.0s
移动到放置位
松开夹爪
回观测位
再次闭合夹爪
等待下一次 d
```

## 4. 真机抓取模式二：自动循环

目的：使用已经调好的检测和偏移矩阵，进行自动循环抓取。

当前现场有效命令：

```bash
cd /home/wenjiajie/code/openarm_yaocao/openarm_teleop-openarm_teleop/vr_teleop-teleop-full/skewer_topdown_2d
conda activate anygrasp

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
  --follower-can can1 \
  --follower-only \
  --sudo-password 888012 \
  --camera-backend opencv \
  --camera-index 8 \
  --black-v-max 80 \
  --min-area-px 250 \
  --target-selection skewer \
  --track-roi-px 90 \
  --no-track-fallback-to-full
```

如果还没有录制放置位，先去掉：

```bash
--place-snapshot outputs/right_observe_record_20260723_192233/follower_position_snapshots.txt
--place-duration 3.0
```

录好放置位后，把 `--place-snapshot` 后面的路径换成你的 `follower_position_snapshots.txt`。脚本只读取最后一条右臂 follower 的 7 个手臂关节作为放置位，snapshot 里的夹爪值会被忽略；移动到放置位时保持抓取上来时的闭合夹爪值，到达放置位后才松开。

启动后程序会：

```text
1. 启动 OpenArm replay 保持进程
2. 回放观测动作
3. 打开右腕相机窗口
4. 到达观测位后等待 1s
5. 自动检测、对齐、抓取、放置、回观测位
6. 回到观测位闭合夹爪后等待 1s，继续下一根
```

自动循环时按键：

```text
d  手动立即触发一轮抓取，通常不需要按
r  清除目标锁定
s  保存当前 RGB
q  退出
Esc 退出
```

每一轮自动抓取流程：

```text
检测黑色涂层段并显示 0.25s
XY 平移到夹爪中心
开爪
下探 4cm，用时 5.0s
最低点保持 0.6s
闭爪 1.2s
闭爪后保持 1.2s
上升 10cm，用时 3.0s
移动到放置位
松开夹爪
回观测位
再次闭合夹爪
等待 1s 后自动开始下一轮
```

如果要回到原来的手动快抓模式，把命令里的这四行去掉：

```bash
--auto-cycle
--auto-start-delay-s 1.0
--auto-cycle-delay-s 1.0
--auto-detect-retry-s 1.0
```

自动循环和手动快抓的区别：

```text
手动快抓：每抓一根都要按 d。
自动循环：到观测位后自动抓；每次回观测位后自动开始下一根。
```

## 5. 真机抓取模式三：单次自动到放置位、不松爪

目的：一次只抓一根。程序到达观测位后等待 1s 自动检测并抓取，抓取后移动到放置位，夹爪保持闭合，不张开夹爪，也不自动抓下一根，方便后面接遥操动作。

```bash
cd /home/wenjiajie/code/openarm_yaocao/openarm_teleop-openarm_teleop/vr_teleop-teleop-full/skewer_topdown_2d
conda activate anygrasp

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
  --follower-can can1 \
  --follower-only \
  --sudo-password 888012 \
  --camera-backend opencv \
  --camera-index 8 \
  --black-v-max 80 \
  --min-area-px 250 \
  --target-selection skewer \
  --track-roi-px 90 \
  --no-track-fallback-to-full
```

执行完成后窗口会停留，终端会提示：

```text
[auto-once] finished. Robot is at place pose and gripper remains closed. Press q/Esc to exit.
```

单次自动和自动循环的区别：

```text
自动循环：抓完到放置位后会松开，然后回观测位闭合，继续下一根。
单次自动：抓完只到放置位，夹爪不松开，机器人停在那里，等你接遥操。
```

终端中应看到类似：

```text
[grasp] waiting full replay duration ...s + settle ...s
```

如果只看到“刚下探就上升”，说明抓取 CSV 被提前打断，需要检查脚本是否为最新版本。

## 6. 常见调参

### 检测不到黑色段

```bash
--black-v-max 100
```

或者降低面积阈值：

```bash
--min-area-px 150
```

如果当前签子带红色套/红色标记，代码默认会在黑色段失败时自动找红色区域：

```bash
--red-fallback
```

红色比较暗时：

```bash
--red-v-min 35
```

误检皮肤或背景红色时：

```bash
--red-s-min 100
```

### 误检背景或夹爪

```bash
--min-area-px 300
--target-selection skewer
```

### XY 对齐过冲

```bash
--xy-correction-scale 0.8
```

还过冲就用：

```bash
--xy-correction-scale 0.6
```

### XY 对齐不够

```bash
--xy-correction-scale 1.1
```

或者重新做第 2 步偏移标定。

### 下探不够深

```bash
--grasp-descent-m 0.05
```

### 下探太快，还没到底就闭爪

```bash
--grasp-descent-duration 6.0
--pre-close-hold-duration 1.0
```

### 闭爪后没夹稳就上升

```bash
--gripper-close-duration 1.5
--gripper-close-hold-duration 1.5
```

### 抬升不够

```bash
--post-grasp-lift-m 0.12
```

## 7. 安全注意

```text
每次执行 --execute 前确认工作空间清空。
不要同时运行多个占用 can1 的 replay 或遥操进程。
第一次使用新参数时手放急停附近。
发现目标检测错位时不要继续抓取；自动模式下立即按 q/Esc 退出。
发现机械臂运动方向反常立刻急停。
```
