# Copyright 2025 Enactic, Inc.
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

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def static_tfs_spawner(context: LaunchContext, arm_type):
    arm_type_str = context.perform_substitution(arm_type)
    moveit_config = (
        MoveItConfigsBuilder(
            "openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description_semantic(file_path=f"config/{arm_type_str}/openarm_bimanual.srdf")
        .joint_limits(file_path=f"config/{arm_type_str}/joint_limits.yaml")
        .robot_description_kinematics(file_path=f"config/{arm_type_str}/kinematics.yaml")
        .to_moveit_configs()
    )
    return generate_static_virtual_joint_tfs_launch(moveit_config).entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm_type", default_value="v20"),
        OpaqueFunction(function=static_tfs_spawner, args=[
                       LaunchConfiguration("arm_type")])
    ])
