# dora-openarm-kinematics

Dora nodes for forward and inverse kinematics on the OpenArm bimanual robot, backed by MuJoCo and [mink](https://github.com/kevinzakka/mink). FK/IK logic lives in the [`openarm_control`](https://github.com/enactic/openarm_control) package, imported as `control`.


## Install

```bash
uv sync
```

## Dora Nodes


### `dora-openarm-fk` — Forward Kinematics

Converts per-arm joint angles to end-effector poses via `mj_forward`.


| | |
|---|---|
| **Inputs** | `position_right`, `position_left` `[{"qpos": float32[8]}]` — joints 1–7 + gripper (flat `float32[8]` also accepted) |
| **Outputs** | `pose_right`, `pose_left` `[{"pose": float32[8]}]` — `[px, py, pz, qw, qx, qy, qz, gripper_value]` |

```
--mode           right | left | bimanual  (default: bimanual)
--frame-right    MuJoCo site/body/geom name for right EE  (default: right_ee_control_point)
--frame-left     MuJoCo site/body/geom name for left EE   (default: left_ee_control_point)
--frame-type-*   body | site | geom  (default: site)
--keyframe       initial keyframe name  (default: home)
--xml            MJCF scene file
```

---

### `dora-openarm-ik` — Differential IK

Solves joint angles from EE pose targets using mink's QP-based differential IK. Both arms share one `mink.Configuration` and are solved in a single QP per step. DOFs not driven by IK outputs are frozen (finger joints, lifter).

| | |
|---|---|
| **Inputs** | `target_right`, `target_left` `[{"pose": float32[7]}]` — EE pose targets; `position` `[{"qpos": float32[16]}]` — optional joint-state sync (flat arrays also accepted); `trigger_right` / `trigger_left` `float32[1]` — gripper pass-through |
| **Outputs** | `position_right`, `position_left` `[{"qpos": float32[8]}]` |

```
--mode           right | left | bimanual  (default: bimanual)
--max-iters      IK iterations per event  (default: 5)
--dt             integration timestep per iteration  (default: 0.5)
--damping        global Tikhonov regularization  (default: 1e-3)
--lm-damping     per-task LM damping  (default: 1e-4)
--posture-cost   posture task weight, 0 = disabled  (default: 0.0)
--pos-cost       position task cost  (default: 1.0)
--ori-cost       orientation task cost  (default: 1.0)
--solver         QP backend  (default: daqp)
--frame-right    site/body name for right EE  (default: right_ee_control_point)
--frame-left     site/body name for left EE   (default: left_ee_control_point)
--keyframe       initial keyframe  (default: home)
--xml            MJCF scene file
```

## Quick Start

### FK — visualise leader arm poses

Reads joint angles from a physical leader arm and publishes end-effector poses. Requires a connected leader device.

```bash
uv run dora build example/dataflow-dummy-fk.yaml --uv
uv run dora run example/dataflow-dummy-fk.yaml --uv
```

---

### FK → IK roundtrip

Pipes FK output directly back into IK to verify the solver round-trips correctly. No physical hardware needed beyond the leader.

```bash
uv run dora build example/dataflow-dummy-ik.yaml --uv
uv run dora run example/dataflow-dummy-ik.yaml --uv
```

**Dataflow:** `leader` → `fk` (joints → poses) → `ik` (poses → joints) → `viewer`

Tune the ik solver parameters in:

```yaml
args: "--mode bimanual --max-iters 5 --dt 0.1 --damping 0.25 --posture-cost 0.01 --lm-damping 0.01"
```

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
