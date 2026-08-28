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

"""dora-rs node that mimics OpenArm for testing."""

import dora
import numpy as np
import pyarrow as pa


def main():
    """Move to the given position and output the current position."""
    n_sensors = 8
    position = pa.array(np.zeros(n_sensors), type=pa.float32())

    initialized = False
    node = dora.Node()
    for event in node:
        if event["type"] != "INPUT":
            continue

        # Main process
        event_id = event["id"]
        if event_id == "request_position":
            node.send_output("position", position)
        elif event_id == "move_position":
            if not initialized:
                initialized = True
                node.send_output("status", pa.array(["ready"]))
            value = event["value"]
            if isinstance(value, pa.StructArray):
                position = value.field("new_position")
            else:
                position = value


if __name__ == "__main__":
    main()
