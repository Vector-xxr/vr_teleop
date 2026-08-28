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

"""dora-rs node to control OpenArm Cell Lifter."""

import argparse
import dora
import openarm_can as oa
import math
import os
import time

import pyarrow as pa

# --- Constants ---
VEL_MAX = 30.0
POS_MIN = 0.0
HOLD_POS_VEL = 1.0

# Threshold for joystick deadzone
JOYSTICK_DEADZONE = 0.15
# Scaling factor for velocity calculation (1.0 - 0.15)
JOYSTICK_RANGE = 0.85

# Torque threshold for anomaly detection (collision / mechanical limit)
TORQUE_LIMIT = 1.5


class PositionUnwrapper:
    """Class to handle wrap-around of multi-turn motor encoders (-4pi to 4pi)."""

    def __init__(self, wrap_range=8.0 * math.pi, wrap_threshold=4.0 * math.pi):
        """Initialize the tracker for wrapped encoder positions."""
        self._wrap_range = wrap_range
        self._wrap_threshold = wrap_threshold
        self._prev_raw = None
        self._continuous_pos = 0.0

    def update(self, current_raw):
        """Update the continuous position estimate from the raw encoder value."""
        if self._prev_raw is None:
            self._prev_raw = current_raw
            self._continuous_pos = current_raw
            return self._continuous_pos

        diff = current_raw - self._prev_raw

        # Unwrap logic for 8pi range
        if diff > self._wrap_threshold:
            diff -= self._wrap_range
        elif diff < -self._wrap_threshold:
            diff += self._wrap_range

        self._continuous_pos += diff
        self._prev_raw = current_raw
        return self._continuous_pos


def _calc_next_elevation(current_elevation, velocity, dt, lead_length):
    return current_elevation + velocity * dt * lead_length / (2.0 * math.pi)


def _calc_elevation_action_from_joystick(
    current_elevation, joystick_y, dt, lead_length, speed_factor=1.0
):
    if joystick_y > JOYSTICK_DEADZONE:
        applied_vel = VEL_MAX * (abs(joystick_y - JOYSTICK_DEADZONE) / JOYSTICK_RANGE)
        applied_vel *= speed_factor
        return _calc_next_elevation(
            current_elevation, applied_vel, dt, lead_length
        ), applied_vel

    elif joystick_y < -JOYSTICK_DEADZONE:
        applied_vel = VEL_MAX * (abs(joystick_y + JOYSTICK_DEADZONE) / JOYSTICK_RANGE)
        applied_vel *= speed_factor
        return _calc_next_elevation(
            current_elevation, -applied_vel, dt, lead_length
        ), applied_vel

    else:
        return None, 0.0


def _dora_main(lifter, args):
    pos_max = (args.screw_length / args.lead_length) * 2.0 * math.pi
    slow_margin = pos_max * 0.05

    node = dora.Node()

    lifter_pos = 0.0
    lifter_tau = 0.0

    joystick_y = 0.0

    for motor in lifter.get_arm().get_motors():
        lifter_pos = motor.get_position()
        lifter_tau = motor.get_torque()

    unwrapper = PositionUnwrapper()

    calibrated = False
    offset_pos = 0.0

    # State management variables
    jammed_direction = None
    hold_pos = 0.0
    hold_elevation = 0.0
    is_stopping = False

    prev_time = time.time()

    for event in node:
        if event["type"] != "INPUT":
            continue

        event_id = event.get("id")

        value = event.get("value")
        if value is None:
            continue

        if event_id == "joystick_y":
            py_value = value[0].as_py()
            joystick_y = py_value
        elif event_id == "command":
            command = event["value"][0].as_py()
            if command == "lifter-up":
                joystick_y = 0.5
            elif command == "lifter-down":
                joystick_y = -0.5
            else:
                joystick_y = 0
        elif event_id == "move_elevation":
            pass  # handled in normal operation phase
        elif event_id == "tick":
            pass
        else:
            continue

        # --- Calibration Phase ---
        if not calibrated:
            lifter.get_arm().posvel_control_all(
                [oa.PosVelParam(q=POS_MIN - 1000.0, dq=VEL_MAX / 5.0)]
            )

            lifter.recv_all(1000)  # Wait up to 1ms for the motor response

            for motor in lifter.get_arm().get_motors():
                lifter_pos = motor.get_position()
                lifter_tau = motor.get_torque()
            if abs(lifter_tau) > 1.0:
                offset_pos = lifter_pos
                calibrated = True
                lifter.get_arm().posvel_control_all(
                    [oa.PosVelParam(q=pos_max, dq=VEL_MAX / 20.0)]
                )
                time.sleep(0.2)
            continue

        # --- Normal Operation Phase ---
        now = time.time()
        dt = now - prev_time
        prev_time = now

        lifter.recv_all()

        for motor in lifter.get_arm().get_motors():
            lifter_pos = motor.get_position()
            lifter_tau = motor.get_torque()
        obs_position = unwrapper.update(lifter_pos) - offset_pos
        # obs: elevation(mm)
        obs_elevation = obs_position / (2.0 * math.pi) * args.lead_length
        node.send_output(
            "elevation_observation", pa.array([obs_elevation], type=pa.float32())
        )

        # Calculate distance to the nearest physical limit (lower 0.0 or upper pos_max)
        distance_to_min = obs_position - 0.0
        distance_to_max = pos_max - obs_position
        distance_to_edge = min(distance_to_min, distance_to_max)

        # Calculate the linear deceleration factor
        if distance_to_edge < slow_margin:
            # Clamp to prevent negative values due to slight sensor noise
            clamped_distance = max(0.0, distance_to_edge)
            # Linear interpolation: 0.25x at the edge, 1.0x at the slow_margin boundary
            speed_factor = 0.25 + 0.75 * (clamped_distance / slow_margin)
        else:
            speed_factor = 1.0

        # --- Elevation (mm) ---
        if event_id == "move_elevation":
            elevation = max(0.0, min(args.screw_length, value[0].as_py()))
            target_position = elevation / args.lead_length * 2.0 * math.pi + offset_pos
            is_stopping = False

            node.send_output(
                "elevation_action",
                pa.array([elevation], type=pa.float32()),
            )

            lifter.get_arm().posvel_control_all(
                [oa.PosVelParam(q=target_position, dq=VEL_MAX * speed_factor)]
            )
            continue

        # --- 1. Jam detection and direction memorization ---
        if abs(lifter_tau) > TORQUE_LIMIT:
            if jammed_direction is None:
                # Memorize the exact position at impact to prevent hunting
                hold_pos = obs_position + offset_pos  # Use the unwrapper position
                hold_elevation = obs_elevation  # obs elevation is calculated from the unwrapper lifter_pos(hold_pos)

                if joystick_y > JOYSTICK_DEADZONE:
                    jammed_direction = "UP"
                elif joystick_y < -JOYSTICK_DEADZONE:
                    jammed_direction = "DOWN"
                else:
                    jammed_direction = "UNKNOWN"
        else:
            # Do not release the lock if the joystick is held in the same direction,
            # even if the torque drops to a normal value
            if jammed_direction == "UP" and joystick_y <= JOYSTICK_DEADZONE:
                jammed_direction = None
            elif jammed_direction == "DOWN" and joystick_y >= -JOYSTICK_DEADZONE:
                jammed_direction = None
            elif jammed_direction == "UNKNOWN":
                jammed_direction = None

        # --- 2. Action determination (One-way block, reverse allow, linear deceleration at limits) ---

        # UP operation
        if joystick_y > JOYSTICK_DEADZONE:
            is_stopping = False
            if jammed_direction in ["UP", "UNKNOWN"]:
                # Hold silently at the memorized position while jammed in the UP direction
                node.send_output(
                    "elevation_action",
                    pa.array([hold_elevation], type=pa.float32()),
                )
                lifter.get_arm().posvel_control_all(
                    [oa.PosVelParam(q=hold_pos, dq=0.0)]
                )

            else:
                action_elevation, applied_vel = _calc_elevation_action_from_joystick(
                    obs_elevation,
                    joystick_y,
                    dt,
                    args.lead_length,
                    speed_factor,
                )

                node.send_output(
                    "elevation_action",
                    pa.array([action_elevation], type=pa.float32()),
                )

                # Drive toward the top stroke limit (positive joystick_y = up)
                lifter.get_arm().posvel_control_all(
                    [oa.PosVelParam(q=pos_max + offset_pos, dq=applied_vel)]
                )

        # DOWN operation
        elif joystick_y < -JOYSTICK_DEADZONE:
            is_stopping = False
            if jammed_direction in ["DOWN", "UNKNOWN"]:
                # Hold silently at the memorized position while jammed in the DOWN direction

                node.send_output(
                    "elevation_action",
                    pa.array([hold_elevation], type=pa.float32()),
                )
                lifter.get_arm().posvel_control_all(
                    [oa.PosVelParam(q=hold_pos, dq=0.0)]
                )
            else:
                action_elevation, applied_vel = _calc_elevation_action_from_joystick(
                    obs_elevation,
                    joystick_y,
                    dt,
                    args.lead_length,
                    speed_factor,
                )

                node.send_output(
                    "elevation_action",
                    pa.array([action_elevation], type=pa.float32()),
                )

                # Drive toward the bottom stroke limit (negative joystick_y = down)
                lifter.get_arm().posvel_control_all(
                    [oa.PosVelParam(q=offset_pos, dq=applied_vel)]
                )

        # STOP (Within deadzone)
        else:
            if not is_stopping:
                # Memorize the exact position at the moment the joystick is released
                hold_pos = obs_position + offset_pos  # Use the unwrapper position
                hold_elevation = obs_elevation  # obs elevation is calculated from the unwrapper position(hold_pos)
                is_stopping = True

            node.send_output(
                "elevation_action",
                pa.array([hold_elevation], type=pa.float32()),
            )
            # Hold position with a small velocity to maintain stiffness, preventing drift due to gravity or disturbances
            lifter.get_arm().posvel_control_all(
                [oa.PosVelParam(q=hold_pos, dq=HOLD_POS_VEL)]
            )


def main():
    """Control the OpenArm Lifter using joystick inputs.

    observation:
    elevation (mm) from the bottom, converted from motor angle (rad) using the lead length.

    action:
    Estimated next elevation (mm), calculated from observation and `applied_vel`.
    `applied_vel` is reduced linearly near the stroke limits (within `SLOW_MARGIN`) to avoid collision.
    `action` is only output when the joystick is operated.
    """
    parser = argparse.ArgumentParser(description="Control the OpenArm Lifter")
    parser.add_argument(
        "--can-interface",
        default=os.getenv("CAN_INTERFACE", "can2"),
        help="The CAN interface name",
    )
    parser.add_argument(
        "--lead-length",
        default=float(os.getenv("LEAD_LENGTH", 5.0)),
        help="Lead screw lead length in mm/rev",
        type=float,
    )
    parser.add_argument(
        "--screw-length",
        default=float(os.getenv("SCREW_LENGTH", 300.0)),
        help="Lead screw stroke length in mm",
        type=float,
    )
    args = parser.parse_args()

    lifter = oa.OpenArm(args.can_interface, enable_fd=True)

    motor_types = [oa.MotorType.DM4310]
    send_ids = [0x0A]
    recv_ids = [0x1A]
    control_modes = [oa.ControlMode.POS_VEL]

    lifter.init_arm_motors(motor_types, send_ids, recv_ids, control_modes)
    lifter.set_callback_mode_all(oa.CallbackMode.STATE)

    lifter.enable_all()
    lifter.recv_all()

    _dora_main(lifter, args)

    # Move the lifter down to the mechanical stop before disabling
    for _ in range(200):
        lifter.get_arm().posvel_control_all(
            [oa.PosVelParam(q=POS_MIN - 1000.0, dq=VEL_MAX / 1.0)]
        )

        lifter.recv_all(1000)  # Wait up to 1ms for the motor response
        time.sleep(0.05)  # Small delay to allow the motor to move

        for motor in lifter.get_arm().get_motors():
            lifter_tau = motor.get_torque()
        if abs(lifter_tau) > 1.0:
            break
        # stops the shutdown homing when abs(lifter_tau) > 1.0

    # Disable motors for safety after exiting the loop
    lifter.disable_all()


if __name__ == "__main__":
    main()
