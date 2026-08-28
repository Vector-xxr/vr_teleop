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


import pytest


import dora_openarm_cell_lifter.main as main_module  # noqa: E402


@pytest.mark.parametrize(
    ("joystick_y", "expected_elevation", "expected_velocity"),
    [
        (1.0, 28.8732414637843, 30.0),
        (-1.0, -18.8732414637843, 30.0),
    ],
)
def test_joystick_y_produces_expected_elevation_action(
    joystick_y, expected_elevation, expected_velocity
):
    action_elevation, applied_vel = main_module._calc_elevation_action_from_joystick(
        current_elevation=5.0,
        joystick_y=joystick_y,
        dt=1.0,
        lead_length=5.0,
    )

    assert action_elevation == pytest.approx(expected_elevation)
    assert applied_vel == pytest.approx(expected_velocity)


@pytest.mark.parametrize("joystick_y", [-0.15, 0.0, 0.15])
def test_deadzone_joystick_y_does_not_produce_elevation_action(joystick_y):
    action_elevation, applied_vel = main_module._calc_elevation_action_from_joystick(
        current_elevation=5.0,
        joystick_y=joystick_y,
        dt=1.0,
        lead_length=5.0,
    )

    assert action_elevation is None
    assert applied_vel == 0.0
