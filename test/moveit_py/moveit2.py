#!/usr/bin/env python3

import rclpy
import math
import os
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped

rclpy.init()

# Load config file
config_file = os.path.join(
    get_package_share_directory("niryo_ned2_dual_gripper_moveit_config"),
    "config",
    "moveit_py_params.yaml",
)

# Create MoveIt instance with config file
moveit_py = MoveItPy(node_name="moveit_py_test", launch_params_filepaths=[config_file])

# Get planning component for arm_1
arm_1 = moveit_py.get_planning_component("arm_1")

# Set start state to current state
arm_1.set_start_state_to_current_state()

# Create pose goal
pose_goal = PoseStamped()
pose_goal.header.frame_id = "arm_1_base_link"
pose_goal.pose.position.x = 0.2
pose_goal.pose.position.y = 0.2
pose_goal.pose.position.z = 0.2


# X-axis facing DOWN (pointing toward ground)
qx = 0.0
qy = 0.7071
qz = 0.0
qw = 0.7071


pose_goal.pose.orientation.x = qx
pose_goal.pose.orientation.y = qy
pose_goal.pose.orientation.z = qz
pose_goal.pose.orientation.w = qw

# Result: quaternion (0.7071, 0, 0, 0.7071)
# Set goal state with pose
arm_1.set_goal_state(pose_stamped_msg=pose_goal, pose_link="arm_1_tool_link")

# Plan
plan_result = arm_1.plan()

# Execute
if plan_result:
    moveit_py.execute(plan_result.trajectory, controllers=[])
    print("Successfully moved arm_1 to target pose")
else:
    print("Planning failed")

rclpy.shutdown()
