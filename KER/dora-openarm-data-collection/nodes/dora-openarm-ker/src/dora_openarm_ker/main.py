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

"""dora-rs node for leader OpenArm KER."""

import argparse
import json
import time
import dora
import pyarrow as pa
import numpy as np
from openarm_ker.ker_stream import KERStream, CMD_STANDBY, CMD_STREAM

QPOS_TYPE = pa.struct([("qpos", pa.list_(pa.float32()))])


def build_qpos_output(qpos: np.ndarray) -> pa.Array:
    """Wrap a qpos array as a length-1 StructArray: [{"qpos": [...]}]."""
    return pa.array([{"qpos": qpos}], type=QPOS_TYPE)


# ==============================================================================
# Filters & Math Utilities
# ==============================================================================
def map_range(x, in_min, in_max, out_min, out_max):
    """Map a value from one range to another, with clipping."""
    if in_max == in_min:
        return out_min
    x = max(min(x, in_max), in_min) if in_min < in_max else max(min(x, in_min), in_max)
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class OnlineHampelFilter:
    """Real-time Hampel filter to eliminate hardware spikes with zero phase delay."""

    def __init__(self, window_size=5, n_sigmas=3.0, min_threshold=5.0):
        """Initialize the Hampel filter parameters and historical buffer."""
        self.window_size = window_size
        self.n_sigmas = n_sigmas
        self.min_threshold = min_threshold
        self.scale_factor = 1.4826
        self.history = None

    def process(self, current_values: list[float]) -> list[float]:
        """Process streaming multi-channel data to detect and suppress sudden spikes."""
        curr = np.array(current_values, dtype=np.float64)

        if self.history is None:
            self.history = np.tile(curr, (self.window_size, 1))
            return current_values

        medians = np.median(self.history, axis=0)
        mads = np.median(np.abs(self.history - medians), axis=0)

        thresholds = np.maximum(
            self.n_sigmas * self.scale_factor * mads, self.min_threshold
        )

        is_outlier = np.abs(curr - medians) > thresholds
        if np.any(is_outlier):
            for idx in np.where(is_outlier)[0]:
                print(
                    f"[KER Filter] Spike suppressed at J{idx + 1}! Raw: {curr[idx]:.2f} -> Fixed: {medians[idx]:.2f}"
                )

        filtered_values = np.where(is_outlier, medians, curr)

        self.history[:-1] = self.history[1:]
        self.history[-1] = filtered_values

        return filtered_values.tolist()


# ==============================================================================
# Core Processor
# ==============================================================================
class KerPoseProcessor:
    """Processes raw angles and applies filters without modifying the physical scale."""

    def __init__(self, use_hampel: bool = False):
        """Initialize the pose processor with an optional Hampel filter."""
        self.hampel = (
            OnlineHampelFilter(window_size=5, n_sigmas=3.0, min_threshold=5.0)
            if use_hampel
            else None
        )

    def process(self, raw_angles: list[float]) -> tuple[list[float], list[float]]:
        """Filter raw encoder degrees and transform them into left/right radian lists."""
        filtered = self.hampel.process(raw_angles) if self.hampel else raw_angles

        grip_r_deg = map_range(filtered[7], 0.0, -60.0, -60.0, 10.0)
        grip_l_deg = map_range(filtered[15], 0.0, 60.0, 60.0, -10.0)

        # Validated vs MuJoCo: KER wrist channels J6/J7 are swapped vs OpenArm.
        right7 = list(filtered[:7])
        left7 = list(filtered[8:15])
        right7[5], right7[6] = right7[6], right7[5]
        left7[5], left7[6] = left7[6], left7[5]

        pos_right = np.deg2rad(right7).tolist()
        pos_right.append(np.deg2rad(grip_r_deg))

        pos_left = np.deg2rad(left7).tolist()
        pos_left.append(np.deg2rad(grip_l_deg))

        return pos_right, pos_left


# ==============================================================================
# Main Node App
# ==============================================================================
def main():
    """Act OpenArm KER as a leader of OpenArm."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--hampel", action="store_true", help="Enable Hampel filter")
    args = parser.parse_args()

    print("Connecting to KER device...")
    stream = KERStream()
    stream.connect()
    stream.send_command(CMD_STREAM)

    print("\n=== Verified Device Metadata ===")
    print(f" Hardware : {stream.metadata.get('hw')}")
    print(f" Firmware : {stream.metadata.get('fw')}")
    print(f" Updated  : {stream.metadata.get('updated')}")
    print("================================\n")

    print("Initializing dora-rs node...")
    node = dora.Node()
    processor = KerPoseProcessor(use_hampel=args.hampel)

    print("KER Leader Node Running successfully.\n")
    ### send metadata
    node.send_output(
        "metadata",
        pa.array([json.dumps(stream.metadata)]),
        {"timestamp": time.time_ns()},
    )
    try:
        for event in node:
            if event["type"] == "ERROR":
                print(f"Dora error occurred: {event['error']}")
                break

            elif event["type"] == "STOP":
                print("Received STOP event from dora. Shutting down...")
                break

            if event["type"] != "INPUT":
                continue

            data = stream.recv()
            if data is None:
                continue

            pos_right, pos_left = processor.process(data["angles"])

            ts = {"timestamp": time.time_ns()}

            node.send_output(
                "follower_position_right",
                build_qpos_output(np.array(pos_right, dtype=np.float32)),
                ts,
            )
            node.send_output(
                "follower_position_left",
                build_qpos_output(np.array(pos_left, dtype=np.float32)),
                ts,
            )

            # enc_val = data["encoder_value"]
            # enc_btn = data["encoder_button"]
            #
            # node.send_output("encoder_value", pa.array([enc_val], type=pa.int32()), ts)
            # node.send_output(
            #     "encoder_button", pa.array([int(enc_btn)], type=pa.int32()), ts
            # )
            #
    except KeyboardInterrupt:
        pass

    finally:
        try:
            print("\nShutting down: Sending STANDBY command...")
            stream.send_command(CMD_STANDBY)
            time.sleep(0.1)
        except Exception:
            pass
        stream.close()
        print("KER Leader Node Disconnected safely.")


if __name__ == "__main__":
    main()
