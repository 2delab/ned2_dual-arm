#!/usr/bin/env python3

import rclpy
import os
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from rclpy.action.client import ActionClient
from rclpy.node import Node
from niryo_ned_ros2_interfaces.action import Tool
from niryo_ned_ros2_interfaces.msg import ToolCommand

rclpy.init()

# Create node for action client
node = Node("demo1_node")

# Load config file
config_file = os.path.join(
    get_package_share_directory("niryo_ned2_dual_arm_moveit_config"),
    "config",
    "moveit_py_params.yaml",
)

# Create MoveIt instance
moveit_py = MoveItPy(node_name="demo1_test", launch_params_filepaths=[config_file])

# Get planning component for arm_1
arm_1 = moveit_py.get_planning_component("arm_1")
arm_1.set_start_state_to_current_state()

# Create pose goal
pose_goal = PoseStamped()
pose_goal.header.frame_id = "arm_1_base_link"
pose_goal.pose.position.x = 0.35
pose_goal.pose.position.y = 0.2
pose_goal.pose.position.z = 0.2

pose_goal.pose.orientation.x = 0.0
pose_goal.pose.orientation.y = 0.7071
pose_goal.pose.orientation.z = 0.0
pose_goal.pose.orientation.w = 0.7071

# Phase 1: Move to first pose + Open gripper
arm_1.set_goal_state(pose_stamped_msg=pose_goal, pose_link="arm_1_tool_link")
plan_result = arm_1.plan()

if plan_result:
    moveit_py.execute(plan_result.trajectory, controllers=[])
    print("Phase 1: Successfully moved arm_1 to first pose")
else:
    print("Phase 1: Planning failed")

# Open gripper
action_client = ActionClient(
    node, Tool, "/arm_1/niryo_robot_tools_commander/action_server"
)
tool_cmd = ToolCommand()
tool_cmd.cmd_type = 1  # OPEN_GRIPPER
tool_cmd.tool_id = 11  # Gripper1
tool_cmd.max_torque_percentage = 80
tool_cmd.hold_torque_percentage = 30

goal = Tool.Goal()
goal.cmd = tool_cmd

action_client.send_goal_async(goal)
print("Phase 1: Open gripper command sent (async)")

# Phase 2: Move to second pose + Close gripper
arm_1.set_start_state_to_current_state()

pose_goal = PoseStamped()
pose_goal.header.frame_id = "arm_1_base_link"
pose_goal.pose.position.x = 0.35
pose_goal.pose.position.y = -0.2
pose_goal.pose.position.z = 0.2

pose_goal.pose.orientation.x = 0.0
pose_goal.pose.orientation.y = 0.7071
pose_goal.pose.orientation.z = 0.0
pose_goal.pose.orientation.w = 0.7071

arm_1.set_goal_state(pose_stamped_msg=pose_goal, pose_link="arm_1_tool_link")
plan_result = arm_1.plan()

if plan_result:
    moveit_py.execute(plan_result.trajectory, controllers=[])
    print("Phase 2: Successfully moved arm_1 to second pose")
else:
    print("Phase 2: Planning failed")

# Close gripper
tool_cmd = ToolCommand()
tool_cmd.cmd_type = 2  # CLOSE_GRIPPER
tool_cmd.tool_id = 11  # Gripper1
tool_cmd.max_torque_percentage = 80
tool_cmd.hold_torque_percentage = 30

goal = Tool.Goal()
goal.cmd = tool_cmd

action_client.send_goal_async(goal)
print("Phase 2: Close gripper command sent (async)")

rclpy.shutdown()
