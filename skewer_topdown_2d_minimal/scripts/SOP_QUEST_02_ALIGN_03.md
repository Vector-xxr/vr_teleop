# Quest 02 -> 视觉抓取 -> 03 操作 SOP

这个流程需要的入口脚本、动作 CSV、视觉脚本、OpenArm 控制代码都已经放进：

```text
/home/lan/openarm_teleop1/skewer_topdown_2d_minimal
```

这里不依赖软链接。运行入口、动作文件、vendor 控制脚本都是物理文件。

## 包内文件

- 入口脚本：`scripts/run_quest_02_align_03.sh`
- 动作 1：`recordings/sessions/quest/02/real_teleop_dual_merged.csv`
- 动作 2：`recordings/sessions/quest/03/real_teleop_dual_merged.csv`
- 视觉抓取脚本：`scripts/align_once_move.py`
- 视觉检测与标定辅助：`scripts/auto_calibrate_pixel_to_motion.py`、`run_skewer_2d_servo.py`
- 视觉标定配置：`config/skewer_align_config.json`
- OpenArm 控制代码：`vendor/openarm_inplace`
- OpenArm FK/IK：`vendor/openarm_vision_grasp_yolo`
- OpenArm URDF/CAN 依赖：`vendor/openarm_description`、`vendor/openarm_can`
- 默认回放启动位姿：`vendor/motor_p11.txt`

## 默认运行

进入包目录：

```bash
cd /home/lan/openarm_teleop1/skewer_topdown_2d_minimal
OPENARM_SUDO_PASSWORD=admin scripts/run_quest_02_align_03.sh
```

默认参数：

- 左臂 follower CAN：`follower_left`
- 右臂 follower CAN：`follower_right`
- 右臂相机：OpenCV `/dev/video16`
- 动作 1：`recordings/sessions/quest/02/real_teleop_dual_merged.csv`
- 动作 2：`recordings/sessions/quest/03/real_teleop_dual_merged.csv`

执行顺序：

1. 启动一个常驻双臂 `dual_replay_control` 控制进程。
2. 在同一个控制进程里播放动作 1。
3. 读取动作 1 最后一行右臂位置，作为视觉抓取起始位。
4. 执行右臂视觉检测、XY 对齐、下降、夹爪闭合、抬升。
5. 读取动作 2 第一行右臂位置，抓取完成后移动到这个位置。
6. 在同一个控制进程里继续播放动作 2。
7. 全部结束后统一 shutdown。

注意：机器人/CAN 控制始终在同一个 C++ 控制进程里，不会在动作 1、抓取、动作 2 之间断开。

## 替换相机参数

右臂相机是 `/dev/videoN` 时，改 `OPENARM_RIGHT_CAMERA_INDEX`：

```bash
OPENARM_SUDO_PASSWORD=admin \
OPENARM_RIGHT_CAMERA_INDEX=16 \
scripts/run_quest_02_align_03.sh
```

如果临时换成 `/dev/video8`：

```bash
OPENARM_SUDO_PASSWORD=admin \
OPENARM_RIGHT_CAMERA_INDEX=8 \
scripts/run_quest_02_align_03.sh
```

默认相机后端是 OpenCV。如果需要显式指定：

```bash
OPENARM_SUDO_PASSWORD=admin \
OPENARM_RIGHT_CAMERA_BACKEND=opencv \
OPENARM_RIGHT_CAMERA_INDEX=16 \
scripts/run_quest_02_align_03.sh
```

脚本启动时会打印实际使用的相机：

```text
[quest-align] right camera: opencv /dev/video16
```

## 替换回放动作

方式 1：直接覆盖包内 CSV。

```text
recordings/sessions/quest/02/real_teleop_dual_merged.csv
recordings/sessions/quest/03/real_teleop_dual_merged.csv
```

方式 2：不覆盖文件，运行时指定新动作路径：

```bash
OPENARM_SUDO_PASSWORD=admin \
OPENARM_ACTION1_CSV=/abs/path/to/action_02.csv \
OPENARM_ACTION2_CSV=/abs/path/to/action_03.csv \
scripts/run_quest_02_align_03.sh
```

脚本会自动读取：

- 动作 1 最后一行右臂 `q8..q15`，生成视觉抓取起始位。
- 动作 2 第一行右臂 `q8..q14`，作为抓取后的移动目标位。

自动生成的桥接文件在：

```text
recordings/sessions/quest/_generated_align_bridge/
```

## 替换 CAN 映射

默认：

- 左臂：`follower_left`
- 右臂：`follower_right`

如果左右反了，运行时覆盖：

```bash
OPENARM_SUDO_PASSWORD=admin \
OPENARM_LEFT_FOLLOWER_CAN=follower_left \
OPENARM_RIGHT_FOLLOWER_CAN=follower_right \
scripts/run_quest_02_align_03.sh
```

脚本启动时会打印：

```text
[quest-align] CAN: left=follower_left right=follower_right
```

## 调节视觉识别参数

常用视觉参数：

```bash
OPENARM_SUDO_PASSWORD=admin \
OPENARM_RIGHT_CAMERA_INDEX=16 \
OPENARM_BLACK_V_MAX=80 \
OPENARM_MIN_AREA_PX=250 \
OPENARM_TARGET_SELECTION=skewer \
OPENARM_TRACK_ROI_PX=90 \
scripts/run_quest_02_align_03.sh
```

参数说明：

- `OPENARM_BLACK_V_MAX`：黑色区域亮度阈值。黑色识别不到时调大，比如 `100`。
- `OPENARM_MIN_AREA_PX`：黑色候选最小面积。误检杂物时调大，比如 `300`；目标太小时调小，比如 `150`。
- `OPENARM_TARGET_SELECTION`：目标选择方式。推荐 `skewer`。
- `OPENARM_TRACK_ROI_PX`：追踪 ROI 半径。目标移动后丢失时调大，比如 `130`。
- `OPENARM_TRACK_FALLBACK_TO_FULL`：追踪失败后是否允许回到全图搜索。多签子场景建议 `0`，避免换目标。

示例：黑色太浅，调高阈值：

```bash
OPENARM_SUDO_PASSWORD=admin \
OPENARM_BLACK_V_MAX=100 \
scripts/run_quest_02_align_03.sh
```

示例：误检太多，增大面积：

```bash
OPENARM_SUDO_PASSWORD=admin \
OPENARM_MIN_AREA_PX=300 \
scripts/run_quest_02_align_03.sh
```

示例：目标容易跑出追踪框：

```bash
OPENARM_SUDO_PASSWORD=admin \
OPENARM_TRACK_ROI_PX=130 \
scripts/run_quest_02_align_03.sh
```

如果要传 `align_once_move.py` 的其它低频参数，用：

```bash
OPENARM_SUDO_PASSWORD=admin \
OPENARM_ALIGN_EXTRA_ARGS="--roi-top 0.10 --roi-bottom 0.85 --red-min-area-px 100" \
scripts/run_quest_02_align_03.sh
```

视觉标定矩阵和夹爪中心像素在：

```text
config/skewer_align_config.json
```

更换相机安装位置、相机标定、夹爪中心像素时，需要更新这个 JSON。

## 调节抓取动作参数

常用抓取动作参数：

```bash
OPENARM_SUDO_PASSWORD=admin \
OPENARM_ALIGN_MAX_MOVE_MM=180 \
OPENARM_ALIGN_XY_CORRECTION_SCALE=1 \
OPENARM_GRASP_DESCENT_M=0.04 \
OPENARM_GRASP_DESCENT_DURATION=5.0 \
OPENARM_POST_GRASP_LIFT_M=0.10 \
OPENARM_POST_GRASP_LIFT_DURATION=3.0 \
OPENARM_PLACE_DURATION=3.0 \
OPENARM_GRIPPER_OPEN=-0.5 \
OPENARM_GRIPPER_CLOSE=0.1 \
scripts/run_quest_02_align_03.sh
```

参数说明：

- `OPENARM_ALIGN_MAX_MOVE_MM`：单次 XY 最大修正距离，默认 `180`。
- `OPENARM_ALIGN_XY_CORRECTION_SCALE`：XY 修正比例，默认 `1`。如果过冲，可调到 `0.7` 或 `0.8`。
- `OPENARM_GRASP_DESCENT_M`：抓取下降高度，单位米，默认 `0.04`。
- `OPENARM_GRASP_DESCENT_DURATION`：下降时间，默认 `5.0` 秒。
- `OPENARM_POST_GRASP_LIFT_M`：抓取后抬升高度，单位米，默认 `0.10`。
- `OPENARM_POST_GRASP_LIFT_DURATION`：抬升时间，默认 `3.0` 秒。
- `OPENARM_PLACE_DURATION`：抓完移动到动作 2 起始右臂位姿的时间，默认 `3.0` 秒。
- `OPENARM_GRIPPER_OPEN`：夹爪打开值，默认 `-0.5`。
- `OPENARM_GRIPPER_CLOSE`：夹爪闭合值，默认 `0.1`。

## 正常输出

启动阶段应看到：

```text
[quest-align] package root: /home/lan/openarm_teleop1/skewer_topdown_2d_minimal
[quest-align] action1: ...
[quest-align] action2: ...
[quest-align] CAN: left=follower_left right=follower_right
[quest-align] right camera: opencv /dev/video16
```

动作播放阶段应看到：

```text
[quest-align] UDP play|... -> port 47920
[quest-align][status] accepted|...
[quest-align][status] started|...
[quest-align][status] finished|cycle=...
```

如果动作文件没有执行，重点看有没有 `accepted` 或 `started`。

- 没有 `accepted` / `started`：UDP command server 没收到命令，或端口被占用。
- 有 `load_failed`：动作 CSV 路径错误，或 CSV 格式不对。
- 有 `safety`：控制进程触发安全保护，需要看对应日志和机械臂状态。
