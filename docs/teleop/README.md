# 遥操文档（统一目录）

本目录汇总 `vr_control` 工程内**全部遥操相关链路**的流程说明与技术说明。  
工程根：`/home/knowin/Desktop/vr_control`（下文相对路径均相对此根）。

日常操作步骤仍以根目录 [SOP.md](../../SOP.md) 为准；本目录侧重**链路拓扑、数据流、模块职责**。

---

## 文档索引

### 总览

| 文档 | 内容 |
| --- | --- |
| [01_OVERVIEW.md](01_OVERVIEW.md) | 全部遥操链路一览、推荐日常路径、文档/代码入口对照 |

### 流程链路

| 文档 | 内容 |
| --- | --- |
| [02_PIPELINE_VR_LIVE.md](02_PIPELINE_VR_LIVE.md) | Quest VR 真机/仿真实时遥操（双臂/单臂、启动轨迹） |
| [03_PIPELINE_PANELS.md](03_PIPELINE_PANELS.md) | 录制面板、TCP 点动、主从锁定、画圆 |
| [04_PIPELINE_CAMERA_BODY.md](04_PIPELINE_CAMERA_BODY.md) | FPV 相机 Bridge、Body Pose / HybrIK |
| [05_PIPELINE_REPLAY.md](05_PIPELINE_REPLAY.md) | 动作录制产物、双臂/单臂回放、回放面板 |
| [06_PIPELINE_RELATED.md](06_PIPELINE_RELATED.md) | 旁路链路：视觉抓取、Fairino 遗留、其它入口 |

### 技术说明

| 文档 | 内容 |
| --- | --- |
| [07_TECH_CONTROL_STACK.md](07_TECH_CONTROL_STACK.md) | C++ 控制栈：HM → IK → JRG → MIT → CAN/MuJoCo |
| [08_TECH_FRAMES_MODES.md](08_TECH_FRAMES_MODES.md) | 坐标系、遥操模式、soft-reach、夹爪状态机 |
| [09_TECH_COUPLE_JOG.md](09_TECH_COUPLE_JOG.md) | 主从锁定与面板点动实现要点 |
| [10_PORTS_ENV.md](10_PORTS_ENV.md) | 端口速查 + 关键环境变量 |

### 其它仓库内文档（未搬迁，交叉引用）

| 文档 | 说明 |
| --- | --- |
| [../../SOP.md](../../SOP.md) | 操作 SOP + 环境变量大表 |
| [../../quest/SOP_QUEST_HUMAN_MIMIC.md](../../quest/SOP_QUEST_HUMAN_MIMIC.md) | `human_mimic` 细则 |
| [../../openarm_teleop/openarm_inplace/docs/TECHNICAL_DESIGN.md](../../openarm_teleop/openarm_inplace/docs/TECHNICAL_DESIGN.md) | inplace 录制/回放/安全层设计 |
| [../VR_TELEOP_PIPELINE.md](../VR_TELEOP_PIPELINE.md) / [../VR_TELEOP_TECH.md](../VR_TELEOP_TECH.md) | 旧路径跳转页（内容已并入本目录） |
