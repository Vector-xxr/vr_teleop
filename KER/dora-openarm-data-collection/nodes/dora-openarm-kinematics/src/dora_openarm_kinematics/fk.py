# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Dora node: forward kinematics for OpenArm bimanual setup.

Receives per-arm 8-value joint position arrays and publishes the
end-effector pose for the arm that was updated.

Inputs:
  position_right – [{"qpos": float32[8]}]  right arm joints 1–7 + gripper
  position_left  – [{"qpos": float32[8]}]  left  arm joints 1–7 + gripper
  Flat float32 arrays are also accepted.

Outputs:
  pose_right – [{"pose": float32[8]}]  [px, py, pz, qw, qx, qy, qz, gripper_value]
  pose_left  – [{"pose": float32[8]}]  [px, py, pz, qw, qx, qy, qz, gripper_value]
  status     – ["ready"] on startup
"""

import argparse
import time

import dora
import numpy as np
import pyarrow as pa

from openarm_control import Kinematics, register_common_args, setup_from_args

_POSE_STRUCT_TYPE = pa.struct({"pose": pa.list_(pa.float32())})


def build_pose_output(pose: np.ndarray) -> pa.Array:
    """Wrap a pose array as a length-1 StructArray: [{"pose": [...]}]."""
    return pa.array([{"pose": pose}], type=_POSE_STRUCT_TYPE)


def extract_values(value: pa.Array, key: str) -> np.ndarray:
    """Read `key` from a length-1 StructArray, or a flat array as-is."""
    if pa.types.is_struct(value.type):
        value = value.field(key)[0].values
    return np.array(value, dtype=np.float32)


def _run(args: argparse.Namespace) -> None:
    kin = Kinematics(setup_from_args(args))

    node = dora.Node()
    node.send_output("status", pa.array(["ready"]))

    step_count = 0
    t_loop_start = time.perf_counter()

    for event in node:
        if event["type"] != "INPUT":
            continue

        eid = event["id"]
        if eid == "position_right" and "right" in kin.setup.sides:
            values = extract_values(event["value"], "qpos")
            if values.shape != (8,):
                print(
                    f"Warning: expected position_right[8], got {values.shape}. Skipping."
                )
                continue
            t0 = time.perf_counter()
            pose = kin.fk("right", values)
            gripper_value = values[7]  # last value is gripper
            node.send_output(
                "pose_right",
                build_pose_output(
                    np.concatenate([pose, np.array([gripper_value], dtype=np.float32)])
                ),
                {"timestamp": time.time_ns()},
            )

        elif eid == "position_left" and "left" in kin.setup.sides:
            values = extract_values(event["value"], "qpos")
            if values.shape != (8,):
                print(
                    f"Warning: expected position_left[8], got {values.shape}. Skipping."
                )
                continue
            t0 = time.perf_counter()
            pose = kin.fk("left", values)
            gripper_value = values[7]  # last value is gripper
            node.send_output(
                "pose_left",
                build_pose_output(
                    np.concatenate([pose, np.array([gripper_value], dtype=np.float32)])
                ),
                {"timestamp": time.time_ns()},
            )

        else:
            continue

        step_count += 1
        step_ms = (time.perf_counter() - t0) * 1e3
        elapsed = time.perf_counter() - t_loop_start
        hz = step_count / elapsed if elapsed > 0 else 0.0
        if step_count % 2000 == 0:
            print(f"[fk] step={step_count:5d}  step={step_ms:.2f}ms  hz={hz:.1f}")


def main() -> None:
    """Forward kinematics for OpenArm."""
    parser = argparse.ArgumentParser(
        description="FK dora node – OpenArm end-effector poses from joint angles"
    )
    register_common_args(parser)
    args = parser.parse_args()
    _run(args)


if __name__ == "__main__":
    main()
