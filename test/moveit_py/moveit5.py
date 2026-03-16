#!/usr/bin/env python3

import rclpy
import math
import os
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

rclpy.init()

# Load config file
config_file = os.path.join(
    get_package_share_directory("niryo_ned2_dual_arm_moveit_config"),
    "config",
    "moveit_py_params.yaml",
)

# Create MoveIt instance with config file
moveit_py = MoveItPy(node_name="moveit_py_test", launch_params_filepaths=[config_file])

# Get planning component for dual arm group
dual_arm = moveit_py.get_planning_component("dual")

# Get robot model
robot_model = moveit_py.get_robot_model()

# Phase 1: Move to 0 degrees
dual_arm.set_start_state_to_current_state()
robot_state = RobotState(robot_model)
robot_state.set_joint_group_positions("arm_1", [0, 0, 0, 0, 0, 0])
robot_state.set_joint_group_positions("arm_2", [0, 0, 0, 0, 0, 0])
dual_arm.set_goal_state(robot_state=robot_state)

plan_result = dual_arm.plan()
if plan_result:
    moveit_py.execute(plan_result.trajectory, controllers=[])

# Phase 2: Move to 50 degrees
dual_arm.set_start_state_to_current_state()
robot_state = RobotState(robot_model)
target_radians = math.radians(50.0)
robot_state.set_joint_group_positions("arm_1", [target_radians, 0, 0, 0, 0, 0])
robot_state.set_joint_group_positions("arm_2", [target_radians, 0, 0, 0, 0, 0])
dual_arm.set_goal_state(robot_state=robot_state)

plan_result = dual_arm.plan()
if plan_result:
    moveit_py.execute(plan_result.trajectory, controllers=[])

rclpy.shutdown()
