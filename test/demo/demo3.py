#!/usr/bin/env python3
import rclpy
import os
import time
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from rclpy.action.client import ActionClient
from rclpy.node import Node
from niryo_ned_ros2_interfaces.action import Tool
from niryo_ned_ros2_interfaces.msg import ToolCommand

rclpy.init()
# Create node for action client
node = Node("demo3_node")
# Load config file
config_file = os.path.join(
    get_package_share_directory("niryo_ned2_dual_gripper_moveit_config"),
    "config",
    "moveit_py_params.yaml",
)
# Create MoveIt instance
moveit_py = MoveItPy(node_name="demo3_test", launch_params_filepaths=[config_file])
# Get planning component for arm_1
arm_1 = moveit_py.get_planning_component("arm_1")
arm_1.set_start_state_to_current_state()

# Create action client for gripper
action_client = ActionClient(
    node, Tool, "/arm_1/niryo_robot_tools_commander/action_server"
)
# Create pose goal for Phase 1 - Approach pose (joint-space plan)
pose_goal = PoseStamped()
pose_goal.header.frame_id = "arm_1_base_link"
pose_goal.pose.position.x = 0.30
pose_goal.pose.position.y = 0.2
pose_goal.pose.position.z = 0.2
pose_goal.pose.orientation.x = 0.0
pose_goal.pose.orientation.y = 0.7071
pose_goal.pose.orientation.z = 0.0
pose_goal.pose.orientation.w = 0.7071
# Phase 1: Plan joint-space motion to approach pose
print("Phase 1: Planning joint-space motion to approach pose (0.30, 0.2, 0.2)")
arm_1.set_goal_state(pose_stamped_msg=pose_goal, pose_link="arm_1_tool_link")
plan_result_phase1 = arm_1.plan()
if plan_result_phase1:
    print("Phase 1: Planning successful")
    moveit_py.execute(plan_result_phase1.trajectory, controllers=[])
    print("Phase 1: Successfully executed joint-space motion to approach pose")
else:
    print("Phase 1: Planning failed")

# Open gripper at approach pose
tool_cmd = ToolCommand()
tool_cmd.cmd_type = 1  # OPEN_GRIPPER
tool_cmd.tool_id = 11  # Gripper1
tool_cmd.max_torque_percentage = 80
tool_cmd.hold_torque_percentage = 30

goal = Tool.Goal()
goal.cmd = tool_cmd

action_client.send_goal_async(goal)
print("Open gripper command sent")
time.sleep(2)  # Wait 2 seconds for gripper to open
# Phase 2: Plan joint-space motion to lower pose
print("\nPhase 2: Planning joint-space motion to lower pose (0.30, 0.2, 0.1)")
arm_1.set_start_state_to_current_state()

pose_goal_phase2 = PoseStamped()
pose_goal_phase2.header.frame_id = "arm_1_base_link"
pose_goal_phase2.pose.position.x = 0.30
pose_goal_phase2.pose.position.y = 0.2
pose_goal_phase2.pose.position.z = 0.1

pose_goal_phase2.pose.orientation.x = 0.0
pose_goal_phase2.pose.orientation.y = 0.7071
pose_goal_phase2.pose.orientation.z = 0.0
pose_goal_phase2.pose.orientation.w = 0.7071

# Plan Phase 2
arm_1.set_goal_state(pose_stamped_msg=pose_goal_phase2, pose_link="arm_1_tool_link")
plan_result_phase2 = arm_1.plan()

if plan_result_phase2:
    print("Phase 2: Planning successful")
    moveit_py.execute(plan_result_phase2.trajectory, controllers=[])
    print("Phase 2: Successfully executed joint-space motion to lower pose")
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
print("Close gripper command sent")
time.sleep(3)  # Wait 3 seconds for gripper to close

# Phase 3: Plan joint-space motion to drop pose
print("\nPhase 3: Planning joint-space motion to drop pose (0.30, -0.2, 0.2)")
arm_1.set_start_state_to_current_state()

pose_goal_phase3 = PoseStamped()
pose_goal_phase3.header.frame_id = "arm_1_base_link"
pose_goal_phase3.pose.position.x = 0.30
pose_goal_phase3.pose.position.y = -0.2
pose_goal_phase3.pose.position.z = 0.2

pose_goal_phase3.pose.orientation.x = 0.0
pose_goal_phase3.pose.orientation.y = 0.7071
pose_goal_phase3.pose.orientation.z = 0.0
pose_goal_phase3.pose.orientation.w = 0.7071

# Plan Phase 3
arm_1.set_goal_state(pose_stamped_msg=pose_goal_phase3, pose_link="arm_1_tool_link")
plan_result_phase3 = arm_1.plan()

if plan_result_phase3:
    print("Phase 3: Planning successful")
    moveit_py.execute(plan_result_phase3.trajectory, controllers=[])
    print("Phase 3: Successfully executed joint-space motion to drop pose")
else:
    print("Phase 3: Planning failed")

# Phase 4: Plan joint-space motion to lower drop pose
print("\nPhase 4: Planning joint-space motion to lower drop pose (0.30, -0.2, 0.1)")
arm_1.set_start_state_to_current_state()

pose_goal_phase4 = PoseStamped()
pose_goal_phase4.header.frame_id = "arm_1_base_link"
pose_goal_phase4.pose.position.x = 0.30
pose_goal_phase4.pose.position.y = -0.2
pose_goal_phase4.pose.position.z = 0.1

pose_goal_phase4.pose.orientation.x = 0.0
pose_goal_phase4.pose.orientation.y = 0.7071
pose_goal_phase4.pose.orientation.z = 0.0
pose_goal_phase4.pose.orientation.w = 0.7071

# Plan Phase 4
arm_1.set_goal_state(pose_stamped_msg=pose_goal_phase4, pose_link="arm_1_tool_link")
plan_result_phase4 = arm_1.plan()

if plan_result_phase4:
    print("Phase 4: Planning successful")
    moveit_py.execute(plan_result_phase4.trajectory, controllers=[])
    print("Phase 4: Successfully executed joint-space motion to lower drop pose")
else:
    print("Phase 4: Planning failed")

# Open gripper at drop pose
tool_cmd = ToolCommand()
tool_cmd.cmd_type = 1  # OPEN_GRIPPER
tool_cmd.tool_id = 11  # Gripper1
tool_cmd.max_torque_percentage = 80
tool_cmd.hold_torque_percentage = 30

goal = Tool.Goal()
goal.cmd = tool_cmd

action_client.send_goal_async(goal)
print("Open gripper command sent")
time.sleep(2)  # Wait 2 seconds for gripper to open

rclpy.shutdown()
