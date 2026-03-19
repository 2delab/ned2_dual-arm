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
    get_package_share_directory("niryo_ned2_dual_gripper_moveit_config"),
    "config",
    "moveit_py_params.yaml",
)

# Create MoveIt instance with config file
moveit_py = MoveItPy(node_name="moveit_py_test", launch_params_filepaths=[config_file])

# Get planning components
arm_1 = moveit_py.get_planning_component("arm_1")
arm_2 = moveit_py.get_planning_component("arm_2")

# Get robot model
robot_model = moveit_py.get_robot_model()
target_radians = math.radians(50.0)

# Plan and execute arm_1
arm_1.set_start_state_to_current_state()
robot_state_1 = RobotState(robot_model)
robot_state_1.set_joint_group_positions("arm_2", [target_radians, 0, 0, 0, 0, 0])
arm_1.set_goal_state(robot_state=robot_state_1)

plan_result_1 = arm_1.plan()
if plan_result_1:
    moveit_py.execute(plan_result_1.trajectory, controllers=[])
    print("Successfully moved arm_1 to 50 degrees")
else:
    print("Planning failed for arm_1")

# Plan and execute arm_2
arm_2.set_start_state_to_current_state()
robot_state_2 = RobotState(robot_model)
robot_state_2.set_joint_group_positions("arm_2", [target_radians, 0, 0, 0, 0, 0])
arm_2.set_goal_state(robot_state=robot_state_2)

plan_result_2 = arm_2.plan()
if plan_result_2:
    moveit_py.execute(plan_result_2.trajectory, controllers=[])
    print("Successfully moved arm_2 to 50 degrees")
else:
    print("Planning failed for arm_2")

rclpy.shutdown()
