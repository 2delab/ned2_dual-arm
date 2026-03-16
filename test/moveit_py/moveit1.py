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

# Get planning component for arm_1
arm_1 = moveit_py.get_planning_component("arm_1")

# Set start state to current state
arm_1.set_start_state_to_current_state()

# Get robot model and create goal state
robot_model = moveit_py.get_robot_model()
robot_state = RobotState(robot_model)

# Set arm_1_joint_1 to 50 degrees (convert to radians)
target_degrees = 50.0
target_radians = math.radians(target_degrees)
robot_state.set_joint_group_positions("arm_1", [target_radians, 0, 0, 0, 0, 0])

# Set goal state
arm_1.set_goal_state(robot_state=robot_state)

# Plan
plan_result = arm_1.plan()

# Execute
if plan_result:
    moveit_py.execute(plan_result.trajectory, controllers=[])
    print(f"Successfully moved arm_1_joint_1 to {target_degrees} degrees")
else:
    print("Planning failed")

rclpy.shutdown()
