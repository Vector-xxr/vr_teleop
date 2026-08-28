# 烧烤签俯视 2D 视觉抓取

本目录是一套独立的烧烤签俯视 2D 抓取流程。目标是稳定落地，而不是追求复杂 3D 点云匹配。

核心思路：

1. 右腕相机从上方俯视烧烤签。
2. 烧烤签可抓取段有黑色涂层；如果黑色段太细，也支持红色套/红色标记 fallback。
3. RGB 图像中优先检测黑色细长区域，失败后自动检测红色标记区域，得到抓取中心像素和签子角度。
4. 将黑色抓取中心对齐到固定的夹爪中心像素。
5. 先不做腕部旋转，直接做 XY 平移对齐。
6. 对齐后执行固定下探、闭爪、上升；后续根据模式选择“放置松开并回观测位”或“到放置位保持闭合”。

当前流程已经支持：

```text
单根/多根烧烤签检测
黑色涂层检测 + 红色标记 fallback
目标锁定，避免多根场景中切换目标
XY 平移对齐
固定下探抓取
支持抓取后放置松开、回观测位继续下一根
支持抓取后到放置位保持闭合，方便接遥操动作
像素到机械臂 XY 的线性/二次标定模型
pkill -f "left_grasp_exec_bundle.*align_once_move.py"
pkill -f "left_grasp_exec_bundle.*run_cpp_real_replay.sh"
pkill -f "left_grasp_exec_bundle.*unilateral_control"

cd /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/left_grasp_exec_bundle

/home/knowin/miniconda3/envs/openarm_teleop1/bin/python scripts/align_once_move.py \
  --arm-side left_arm \
  --config /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/left_grasp_exec_bundle/config/skewer_align_config_left.json \
  --observe-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/left_grasp_exec_bundle/outputs/left_observe_record_20260728_161523/real_follower_left_01.csv \
  --execute \
  --follower-can follower_left \
  --follower-only \
  --sudo-password knowin \
  --server-startup-s 3.0 \
  --camera-backend opencv \
  --camera-index 6 \
  --quick-grasp-on-d \
  --auto-cycle \
  --auto-start-delay-s 1.0 \
  --auto-cycle-delay-s 1.0 \
  --auto-detect-retry-s 1.0 \
  --no-quick-align-angle \
  --quick-detect-preview-s 0.25 \
  --no-quick-verify-after-xy \
  --max-move-mm 180 \
  --max-step-joint-delta-rad 0.85 \
  --xy-correction-scale-x 0.50 \
  --xy-correction-scale-y 1.10 \
  --grasp-descent-m 0.065 \
  --grasp-descent-duration 5.0 \
  --pre-close-hold-duration 0.6 \
  --gripper-close-duration 1.2 \
  --gripper-close-hold-duration 1.2 \
  --post-grasp-lift-m 0.03 \
  --post-grasp-lift-duration 1.5 \
  --post-grasp-retreat-mm 100 \
  --post-grasp-retreat-duration 1.5 \
  --post-grasp-retreat-order after-lift \
  --return-observe-duration 3.0 \
  --max-grasp-joint-delta-rad 2.0 \
  --gripper-open -0.5 \
  --gripper-close 0.1 \
  --target-selection red \
  --red-h-low 12 \
  --red-h-high 168 \
  --red-v-min 25 \
  --red-s-min 70 \
  --red-min-area-px 35 \
  --red-max-area-px 5000 \
  --red-roi-left 0.0 \
  --red-roi-right 1.0 \
  --red-roi-top 0.05 \
  --red-roi-bottom 0.90 \
  --red-max-target-dist-px 300 \
  --no-track-target


cd /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/skewer_topdown_2d_minimal

# 执行顺序：steady -> action1 -> 等待终端按 d -> 左臂视觉抓取 -> action2
/home/knowin/miniconda3/envs/openarm_teleop1/bin/python scripts/align_once_move.py \
  --execute \
  --dual-replay \
  --arm-side left_arm \
  --config /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/left_grasp_exec_bundle/config/skewer_align_config_left.json \
  --steady-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/recordings/tasks/steady_action_on_table.csv \
  --pre-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/recordings/tasks/action1.csv \
  --observe-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/left_grasp_exec_bundle/outputs/left_observe_record_20260728_161523/real_follower_left_01.csv \
  --post-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/recordings/tasks/action2.csv \
  --right-follower-can follower_right \
  --left-follower-can follower_left \
  --sudo-password knowin \
  --server-startup-s 3.0 \
  --camera-backend opencv \
  --camera-index 6 \
  --target-selection red \
  --red-h-low 12 \
  --red-h-high 168 \
  --red-v-min 25 \
  --red-s-min 70 \
  --red-min-area-px 35 \
  --red-max-area-px 5000 \
  --red-roi-left 0.0 \
  --red-roi-right 1.0 \
  --red-roi-top 0.05 \
  --red-roi-bottom 0.90 \
  --red-max-target-dist-px 300 \
  --no-track-target \
  --gripper-open -0.5 \
  --gripper-close 0.1 \
  --initial-transition-duration 3.0 \
  --pre-to-steady-duration 6.0 \
  --pre-to-observe-duration 10.0 \
  --post-replay-transition-duration 10.0 \
  --max-replay-final-error-rad 0.3491 \
  --verify-replay-feedback \
  --abort-on-replay-feedback-stall \
  --max-move-mm 180 \
  --max-step-joint-delta-rad 0.85 \
  --xy-correction-scale-x 0.50 \
  --xy-correction-scale-y 1.10 \
  --quick-grasp-on-d \
  --no-quick-align-angle \
  --no-quick-verify-after-xy \
  --quick-detect-preview-s 0.25 \
  --grasp-descent-m 0.065 \
  --grasp-descent-duration 5.0 \
  --pre-close-hold-duration 0.6 \
  --gripper-close-duration 1.2 \
  --gripper-close-hold-duration 1.2 \
  --post-grasp-lift-m 0.03 \
  --post-grasp-lift-duration 1.5 \
  --post-grasp-retreat-mm 100 \
  --post-grasp-retreat-duration 1.5 \
  --post-grasp-retreat-order after-lift \
  --return-observe-duration 3.0 \
  --max-grasp-joint-delta-rad 2.0 \
  --no-preview

## 命令模式选择

`scripts/align_once_move.py` 现在主要有三种现场使用方式：

```text
手动快抓
  人按 d 才抓一根。
  抓完会移动到放置位、松开、回观测位、闭合。
  适合调试、第一次上真机、确认检测和下探距离。

自动循环
  到观测位后等待 1s，自动检测并抓取。
  抓完会移动到放置位、松开、回观测位、闭合，然后等待 1s 自动抓下一根。
  适合连续抓取多根签子。

单次自动到放置位不松爪
  到观测位后等待 1s，自动检测并抓取。
  抓完移动到放置位，但夹爪保持闭合，不松开，不回观测位，不继续抓。
  适合后面要接人工遥操动作或其他外部动作。
```

三种命令的核心差异：

```text
--quick-grasp-on-d
  打开“快抓流程”。没有自动参数时，需要手动按 d 触发。

--auto-cycle
  在 --quick-grasp-on-d 基础上自动循环触发，不需要手动按 d。

--auto-once
  在 --quick-grasp-on-d 基础上只自动抓一次，抓完去放置位并保持夹爪闭合。

--place-snapshot
  放置位来源。自动循环和单次自动到放置位都建议传这个参数。

--auto-start-delay-s
  到达观测位后多久开始第一次自动检测。现场默认 1.0s。

--auto-cycle-delay-s
  自动循环模式中，回到观测位后多久开始下一轮。只对 --auto-cycle 生效。

--auto-detect-retry-s
  自动检测失败后隔多久重试。机械臂不会在检测失败时移动。
```

## 目录结构

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
    openarm_can/
    openarm_description/assets/robot/openarm_v1.0/urdf/example/v1.urdf
    openarm_vision_grasp_yolo/
    template_library/rgbd_samurai_minimal/01_capture_rgbd.py
  outputs/
    right_observe_record_20260723_135046/
      real_follower_right_03.csv
    right_observe_record_20260723_192233/
      follower_position_snapshots.txt
```

## 当前默认配置

配置文件：

```text
config/skewer_align_config.json
```

里面保存：

```text
target_center_px
pixel_to_base_mm
pixel_to_base_model
quadratic_offset_from_error_coeff
```

`target_center_px` 是夹爪中心在图像中的固定像素。当前为：

```text
348.0,364.0
```

`pixel_to_base_model` 当前使用二次模型：

```text
quadratic_offset_delta
```

脚本 `align_once_move.py` 会自动读取这个配置。正常抓取时不要再手动传 `--pixel-to-base-mm`，除非你想临时测试某个旧矩阵。

## 主要脚本

### 1. `scripts/align_once_move.py`

主抓取脚本。下面三条命令只差在触发方式和抓完后的收尾动作。

#### 1.1 手动快抓：按一次 `d` 抓一根

这个模式最适合调试。程序会先回放观测位并打开相机，之后等待你按 `d`。按 `d` 后执行：检测、XY 对齐、下探抓取、上升、移动到放置位、松开、回观测位、闭合。

```bash
cd /home/wenjiajie/code/drink/skewer_topdown_2d_minimal
conda activate anygrasp

python scripts/align_once_move.py \
  --observe-replay outputs/right_observe_record_20260724_182742/real_follower_right_01.csv \
  --quick-grasp-on-d \
  --no-quick-align-angle \
  --max-move-mm 180 \
  --max-step-joint-delta-rad 0.85 \
  --xy-correction-scale 1 \
  --grasp-descent-m 0.04 \
  --grasp-descent-duration 5.0 \
  --pre-close-hold-duration 0.6 \
  --gripper-close-duration 1.2 \
  --gripper-close-hold-duration 1.2 \
  --post-grasp-lift-m 0.10 \
  --post-grasp-lift-duration 3.0 \
  --place-snapshot outputs/right_observe_record_20260724_182742/follower_position_snapshots.txt \
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
  --target-selection red \
  --red-v-min 35 \
  --red-s-min 100 \
  --red-min-area-px 80 \
  --red-roi-right 0.92 \
  --track-roi-px 90 \
  --no-track-fallback-to-full
```

按键：

```text
d  抓一根
r  清除目标锁定
s  保存当前 RGB
q  退出
Esc 退出
```

#### 1.2 自动循环：无需按键，连续抓取

这是当前现场连续抓取推荐模式：

```text
回放观测位
打开右腕相机
到达观测位后等待 1s
自动检测黑色涂层段并显示检测结果
自动 XY 平移对齐夹爪中心
自动下探、闭爪、上升
自动移动到放置位、松开夹爪、回观测位、再闭合
等待 1s 后继续抓下一根
```

命令：

```bash
cd /home/wenjiajie/code/drink/skewer_topdown_2d_minimal
conda activate anygrasp

python scripts/align_once_move.py \
  --observe-replay outputs/right_observe_record_20260724_182742/real_follower_right_01.csv \
  --quick-grasp-on-d \
  --auto-cycle \
  --auto-start-delay-s 1.0 \
  --auto-cycle-delay-s 1.0 \
  --auto-detect-retry-s 1.0 \
  --no-quick-align-angle \
  --max-move-mm 180 \
  --max-step-joint-delta-rad 0.85 \
  --xy-correction-scale 1 \
  --grasp-descent-m 0.04 \
  --grasp-descent-duration 5.0 \
  --pre-close-hold-duration 0.6 \
  --gripper-close-duration 1.2 \
  --gripper-close-hold-duration 1.2 \
  --post-grasp-lift-m 0.10 \
  --post-grasp-lift-duration 3.0 \
  --place-snapshot outputs/right_observe_record_20260724_182742/follower_position_snapshots.txt \
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

如果还没有录制放置位，先去掉这两行：

```bash
--place-snapshot outputs/right_observe_record_20260724_182742/follower_position_snapshots.txt
--place-duration 3.0
```

录好 `follower_position_snapshots.txt` 后，推荐使用 `--place-snapshot`，脚本会读取最后一条右臂 follower 的 7 个手臂关节作为放置位。snapshot 里的夹爪值会被忽略；移动到放置位时会保持“抓取上来时的闭合夹爪值”，到达放置位后才松开。也可以直接传关节：

```bash
--place-q q1,q2,q3,q4,q5,q6,q7
```

自动循环时按键：

```text
d  手动立即触发一轮抓取，通常不需要按
r  清除目标锁定
s  保存当前 RGB
q  退出
Esc 退出
```

相比手动快抓，自动循环多了这四个参数：

```bash
--auto-cycle
--auto-start-delay-s 1.0
--auto-cycle-delay-s 1.0
--auto-detect-retry-s 1.0
```

#### 1.3 单次自动：到放置位但不松爪

这个模式用于“一次只抓一根签子”：到观测位后等待 1s，自动检测并抓取；抓取上升后移动到放置位，夹爪保持闭合，不张开夹爪，也不会自动抓下一根，方便后面接遥操动作。

```bash
cd /home/wenjiajie/code/drink/skewer_topdown_2d_minimal
conda activate anygrasp

python scripts/align_once_move.py \
  --observe-replay outputs/right_observe_record_20260723_135046/real_follower_right_03.csv \
  --quick-grasp-on-d \
  --auto-once \
  --auto-start-delay-s 1.0 \
  --auto-detect-retry-s 1.0 \
  --no-quick-align-angle \
  --max-move-mm 180 \
  --max-step-joint-delta-rad 0.85 \
  --xy-correction-scale 1 \
  --grasp-descent-m 0.04 \
  --grasp-descent-duration 5.0 \
  --pre-close-hold-duration 0.6 \
  --gripper-close-duration 1.2 \
  --gripper-close-hold-duration 1.2 \
  --post-grasp-lift-m 0.10 \
  --post-grasp-lift-duration 3.0 \
  --place-snapshot outputs/right_observe_record_20260724_182742/follower_position_snapshots.txt \
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

相比自动循环，这个命令有三个关键区别：

```text
1. 使用 --auto-once，不使用 --auto-cycle。
2. 没有 --auto-cycle-delay-s，因为不循环。
3. 抓完只移动到 --place-snapshot 指定的放置位，夹爪保持闭合，不松开、不回观测位。
```

重要参数：

```text
--max-move-mm
  单次 g 允许的最大平移量。建议 60 到 80。

--xy-correction-scale
  XY 修正比例。过冲时

cd /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/skewer_topdown_2d_minimal

/home/knowin/miniconda3/envs/openarm_teleop1/bin/python scripts/align_once_move.py \
  --execute \
  --dual-replay \
  --arm-side left_arm \
  --config /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/left_grasp_exec_bundle/config/skewer_align_config_left.json \
  --steady-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/recordings/tasks/steady_action_on_table.csv \
  --pre-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/recordings/tasks/action1.csv \
  --observe-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/left_grasp_exec_bundle/outputs/left_observe_record_20260728_161523/real_follower_left_01.csv \
  --post-replay /home/knowin/Desktop/vr_control/openarm_teleop/openarm_inplace/recordings/tasks/action2.csv \
  --right-follower-can follower_right \
  --left-follower-can follower_left \
  --sudo-passwor设 0.6 到 0.8；偏小再调回 1.0。

--min-area-px
  黑色目标最小面积。多根/复杂背景建议 250。

--target-selection red
  直接检测红色套/红色标记，不再先找黑色涂层段。

--red-fallback / --no-red-fallback
  默认开启。当前带红色套/红色标记的签子检测不到黑色段时，会自动检测红色区域。

--red-s-min / --red-v-min
  红色检测的 HSV 饱和度和亮度阈值。红色太暗时可降低 --red-v-min；误检皮肤/背景时可提高 --red-s-min。

--track-roi-px
  目标锁定后的搜索半径。多根签子建议 90。

--no-track-fallback-to-full
  锁定目标附近找不到时不回退全图，避免切到另一根签子。

--grasp-descent-duration
  下探时间。现场这版命令用 5.0 秒，避免刚开始下探就进入下一段动作。

--pre-close-hold-duration
  到达最低点后，开爪保持一小段时间再闭爪。

--gripper-close-hold-duration
  闭爪后保持一小段时间，再开始上升。
```

### 2. `scripts/auto_calibrate_pixel_to_motion.py`

像素误差到机械臂 base XY 平移的标定脚本。

它会：

1. 读取观测位 CSV 最后一帧关节。
2. 用 OpenArm FK 得到观测位末端位姿。
3. 根据按键生成相对观测位的 XY 偏移。
4. 用 IK 生成回放 CSV。
5. 移动到该偏移点。
6. 由操作者按 `1/2/3` 指定当前画面中的目标并保存像素误差。
7. 拟合线性模型和二次模型。
8. `--update-align-config` 时写入 `config/skewer_align_config.json`。

大范围二次标定推荐命令：

```bash
cd /home/wenjiajie/code/drink/skewer_topdown_2d_minimal
conda activate anygrasp

python scripts/auto_calibrate_pixel_to_motion.py \
  --observe-replay outputs/right_observe_record_20260727_154012/real_follower_right_02.csv\
  --initial-transition-duration 0.0 \
  --step-mm 15 \
  --max-offset-mm 80 \
  --execute \
  --update-align-config \
  --follower-can can0 \
  --follower-only \
  --sudo-password 888012 \
  --camera-backend opencv \
  --camera-index 8 \
  --black-v-max 80 \
  --min-area-px 250 \
  --target-selection red \
  --red-v-min 35 \
  --red-s-min 100 \
  --red-min-area-px 80 \
  --red-roi-right 0.92 \
  --track-roi-px 110 \
  --live-preview-detect \
  --auto-capture-after-move \
  --target-center-px=353,309
```

标定时只针对一个红色目标：脚本会在中心点和每次移动到位后自动保存当前红色目标。你只需要确认实时红色检测框始终落在同一个目标上，不要按 `2/3/m` 去采其他签子。`1` 只用于手动补采当前点。

标定窗口按键：

```text
w  base +X 移动 step-mm
s  base -X 移动 step-mm
d  base +Y 移动 step-mm
a  base -Y 移动 step-mm
c  回观测中心
1  手动补采当前画面中的同一个红色目标；正常流程不需要按
2..9  只在你明确要多目标标定时才使用；单目标标定不要按
f  拟合当前数据
q  退出并最终拟合
Esc 退出并最终拟合
```

注意：方向键是累计偏移，不需要每次按 `c`。

例如：

```text
w w w d d
```

表示当前偏移为：

```text
X = +30 mm
Y = +20 mm
```

到达一个位置后，脚本会自动保存一次当前红色检测结果。标定一套矩阵时，必须跟踪同一个红色目标在不同机器人偏移下的位置变化。最后按 `q` 退出时会统一拟合并尝试更新配置。

单目标标定时注意：

```text
不要把不同签子混成同一个标定目标。
如果不同位置保存的是不同签子，RMS 会变大，矩阵会乱。
脚本有 RMS 质量门，坏结果不会轻易覆盖 config。
```

建议采样覆盖：

```text
中心
X 正负方向
Y 正负方向
四个对角区域
尽量覆盖实际会抓取的图像范围
```

### 3. `run_skewer_2d_servo.py`

纯视觉检测和观测调试脚本。常用于离线检查检测算法。

离线检测一张图：

```bash
cd /home/wenjiajie/code/drink/skewer_topdown_2d_minimal
conda activate anygrasp

python run_skewer_2d_servo.py \
  --stage detect-image \
  --image /path/to/latest_rgb.png \
  --output-dir outputs/skewer_2d_servo_test \
  --black-v-max 80 \
  --min-area-px 250 \
  --target-selection red
```

输出：

```text
latest_rgb.png
latest_vis.png
latest_mask.png
latest_detection.json
```

观测位回放后实时检测：

```bash
python run_skewer_2d_servo.py \
  --stage observe-and-detect \
  --observe-replay outputs/right_observe_record_20260724_182742/real_follower_right_01.csv \
  --follower-can can0 \
  --follower-only \
  --sudo-password 888012 \
  --camera-backend opencv \
  --camera-index 8 \
  --black-v-max 80 \
  --min-area-px 250 \
  --target-selection red \
  --red-v-min 35 \
  --red-s-min 100 \
  --red-min-area-px 80
```

实时窗口按键：

```text
d  检测
s  保存 RGB 和可视化
a  打印像素修正
q  退出
```

### 4. `scripts/pick_target_center.py`

在保存的图片上点击夹爪中心像素。

```bash
cd /home/wenjiajie/code/drink/skewer_topdown_2d_minimal
conda activate anygrasp

python scripts/pick_target_center.py \
  --image /home/wenjiajie/code/drink/skewer_topdown_2d_minimal/outputs/observe_detect_20260727_145440/detect_20260727_145815/latest_rgb.png
```

鼠标左键点击两个夹爪内侧尖端的中间位置。脚本会打印：

```text
--target-center-px=348.0,364.0
```

如果需要更新配置，手动修改：

```text
config/skewer_align_config.json
```

### 5. `scripts/record_right_observe_with_camera.py`

录制右臂观测动作，同时打开右腕相机。

```bash
cd /home/wenjiajie/code/drink/skewer_topdown_2d_minimal
conda activate anygrasp

python scripts/record_right_observe_with_camera.py \
  --openarm-inplace vendor/openarm_inplace \
  --follower-can can0 \
  --camera-backend opencv \
  --camera-index 8 \
  --sudo-password 888012
```

相机窗口按键：

```text
s  保存当前 RGB
q  退出
Esc 退出
```

录制面板里使用：

```text
Start Recording
Stop Recording
Save Current Positions
```

## 推荐现场抓取流程

```text
1. 先跑 run_skewer_2d_servo.py 的 detect-image，确认离线检测没问题。
2. 如果相机/观测位/夹爪中心变了，跑 auto_calibrate_pixel_to_motion.py 重新算偏移矩阵。
3. 调试时先跑“手动快抓”，按 d 抓一根，确认检测、XY、下探、闭爪都正常。
4. 连续抓多根时跑“自动循环”：无需按键，自动完成检测 -> XY 对齐 -> 下探抓取 -> 上升 -> 放置位 -> 松开 -> 回观测位 -> 闭合 -> 下一根。
5. 只抓一根并接遥操时跑“单次自动到放置位不松爪”：自动完成检测 -> XY 对齐 -> 下探抓取 -> 上升 -> 放置位，夹爪保持闭合。
```

## 常见问题

### 检测到左下角杂物或夹爪阴影

提高最小面积：

```bash
--min-area-px 250
```

使用签子选择策略：

```bash
--target-selection red
```

### 多根签子时切换目标

确保使用：

```bash
--track-roi-px 90
--no-track-fallback-to-full
```

第一次 `d` 后目标会锁定。只有按 `r` 才会清除锁定。

### XY 移动过冲

降低：

```bash
--xy-correction-scale 0.8
```

仍过冲就用：

```bash
--xy-correction-scale 0.6
```

### 边缘区域偏差大

重新做大范围标定，或者继续使用二次模型。当前配置已经支持二次模型。

### 抓取高度不对

调节：

```bash
--grasp-descent-m 0.04
```

例如下探更深：

```bash
--grasp-descent-m 0.05
```

### 夹爪开合不对

调节：

```bash
--gripper-open -1.13
--gripper-close 0.1
```

如果方向反了，需要根据实测对调或重新设置开合值。
