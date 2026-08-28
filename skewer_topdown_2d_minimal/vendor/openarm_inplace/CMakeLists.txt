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

cmake_minimum_required(VERSION 3.22)
project(openarm_inplace)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Find external packages
# -----------------------------
find_package(OpenArmCAN QUIET)
if(NOT TARGET OpenArmCAN::openarm_can)
  set(OPENARM_CAN_SOURCE_DIR
      "${CMAKE_CURRENT_SOURCE_DIR}/../openarm_can"
      CACHE PATH "Path to the OpenArmCAN source tree")
  if(EXISTS "${OPENARM_CAN_SOURCE_DIR}/include/openarm/can/socket/openarm.hpp")
    message(STATUS "OpenArmCAN package not found; using local source at ${OPENARM_CAN_SOURCE_DIR}")
    add_library(
      openarm_can_local
      ${OPENARM_CAN_SOURCE_DIR}/src/openarm/can/socket/arm_component.cpp
      ${OPENARM_CAN_SOURCE_DIR}/src/openarm/can/socket/gripper_component.cpp
      ${OPENARM_CAN_SOURCE_DIR}/src/openarm/can/socket/openarm.cpp
      ${OPENARM_CAN_SOURCE_DIR}/src/openarm/canbus/can_device_collection.cpp
      ${OPENARM_CAN_SOURCE_DIR}/src/openarm/canbus/can_socket.cpp
      ${OPENARM_CAN_SOURCE_DIR}/src/openarm/damiao_motor/dm_motor.cpp
      ${OPENARM_CAN_SOURCE_DIR}/src/openarm/damiao_motor/dm_motor_control.cpp
      ${OPENARM_CAN_SOURCE_DIR}/src/openarm/damiao_motor/dm_motor_device.cpp
      ${OPENARM_CAN_SOURCE_DIR}/src/openarm/damiao_motor/dm_motor_device_collection.cpp)
    target_include_directories(openarm_can_local
                               PUBLIC ${OPENARM_CAN_SOURCE_DIR}/include)
    add_library(OpenArmCAN::openarm_can ALIAS openarm_can_local)
  else()
    message(FATAL_ERROR
            "OpenArmCAN was not found and local source is missing. Set OPENARM_CAN_SOURCE_DIR.")
  endif()
endif()
# find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

set(MUJOCO_PYTHON_PREFIX
    "$ENV{HOME}/miniconda3/envs/openarm_teleop1/lib/python3.11/site-packages/mujoco"
    CACHE PATH "Path to the Python mujoco package containing include/ and libmujoco")
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
          HINTS "${MUJOCO_PYTHON_PREFIX}/include")
find_library(MUJOCO_LIBRARY mujoco libmujoco.so.3.9.0
             HINTS "${MUJOCO_PYTHON_PREFIX}")
find_package(glfw3 QUIET)

# -----------------------------
# Create static library
# -----------------------------
add_library(
  openarm_inplace_lib STATIC
  src/controller/dynamics.cpp src/controller/control.cpp
  src/controller/pose_kalman_filter.cpp src/controller/cartesian_controller.cpp
  src/controller/reference_smoother.cpp
  src/controller/human_mimic_kinematics.cpp src/controller/virtual_arm_skeleton.cpp
  src/controller/human_mimic_ik_solver.cpp src/controller/joint_reference_generator.cpp
  src/controller/human_mimic_controller.cpp
  src/controller/skeleton_proximal_controller.cpp
  src/controller/body_skeleton_filter.cpp
  src/openarm_port/openarm_init.cpp src/openarm_port/joint_mapper.cpp
  src/safety/safety_filter.cpp src/io/teleop_io.cpp src/io/quest_pose_receiver.cpp
  src/io/skeleton_hint_receiver.cpp src/io/skeleton_pose_receiver.cpp
  src/io/body_pose_receiver.cpp
  src/io/teleop_trajectory_logger.cpp
  src/hardware/teleop_hardware.cpp src/hardware/mujoco_teleop_hardware.cpp)

target_include_directories(openarm_inplace_lib
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

if(NOT TARGET kdl_parser::kdl_parser)
  # For old kdl-parser such as libkdl-parser-dev 1.14.2 in Debian.
  add_library(kdl_parser::kdl_parser INTERFACE IMPORTED)
  target_link_libraries(kdl_parser::kdl_parser
                        INTERFACE ${kdl_parser_LIBRARIES})
  target_include_directories(kdl_parser::kdl_parser
                             INTERFACE ${kdl_parser_INCLUDE_DIRS})
endif()
target_link_libraries(
  openarm_inplace_lib
  PUBLIC OpenArmCAN::openarm_can Eigen3::Eigen ${orocos_kdl_LIBRARIES}
         kdl_parser::kdl_parser urdfdom::urdfdom_model yaml-cpp::yaml-cpp)

# -----------------------------
# Executables
# -----------------------------
add_executable(gravity_comp control/gravity_compensation.cpp)
add_executable(comm_test control/openarm_communication_test.cpp)
add_executable(unilateral_control control/openarm_unilateral_control.cpp)
add_executable(bilateral_control control/openarm_bilateral_control.cpp)
add_executable(mujoco_panel_control control/openarm_mujoco_panel_control.cpp)
add_executable(dual_replay_control control/openarm_dual_replay_control.cpp)
add_executable(follower_record_control control/openarm_follower_record_control.cpp)
add_executable(gripper_jog control/openarm_gripper_jog.cpp)
add_executable(gripper_teleop_test control/openarm_gripper_teleop_test.cpp)
add_executable(gripper_only_teleop_test control/openarm_gripper_only_teleop_test.cpp)
add_executable(quest_cartesian_control control/openarm_quest_cartesian_control.cpp)

target_link_libraries(gravity_comp PRIVATE openarm_inplace_lib)
target_link_libraries(comm_test PRIVATE openarm_inplace_lib)
target_link_libraries(unilateral_control PRIVATE openarm_inplace_lib)
target_link_libraries(bilateral_control PRIVATE openarm_inplace_lib)
target_link_libraries(mujoco_panel_control PRIVATE openarm_inplace_lib)
target_link_libraries(dual_replay_control PRIVATE openarm_inplace_lib)
target_link_libraries(follower_record_control PRIVATE openarm_inplace_lib)
target_link_libraries(gripper_jog PRIVATE openarm_inplace_lib)
target_link_libraries(gripper_teleop_test PRIVATE openarm_inplace_lib)
target_link_libraries(gripper_only_teleop_test PRIVATE openarm_inplace_lib)
target_link_libraries(quest_cartesian_control PRIVATE openarm_inplace_lib)
if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
  target_include_directories(openarm_inplace_lib PUBLIC ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(openarm_inplace_lib PUBLIC ${MUJOCO_LIBRARY})
else()
  message(FATAL_ERROR "MuJoCo C library not found. Set MUJOCO_PYTHON_PREFIX.")
endif()
if(TARGET glfw)
  target_link_libraries(openarm_inplace_lib PUBLIC glfw)
elseif(TARGET glfw3)
  target_link_libraries(openarm_inplace_lib PUBLIC glfw3)
else()
  target_link_libraries(openarm_inplace_lib PUBLIC glfw GL)
endif()
