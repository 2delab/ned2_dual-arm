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
node = Node("demo2_node")

# Load config file
config_file = os.path.join(
    get_package_share_directory("niryo_ned2_dual_gripper_moveit_config"),
    "config",
    "moveit_py_params.yaml",
)

# Create MoveIt instance
moveit_py = MoveItPy(node_name="demo2_test", launch_params_filepaths=[config_file])

# Get planning component for arm_1
arm_1 = moveit_py.get_planning_component("arm_1")

# Create action client
action_client = ActionClient(
    node, Tool, "/arm_1/niryo_robot_tools_commander/action_server"
)

# Pickup poses list (flexible - add/remove as needed)
pickup_poses = [
    (0.40, 0.2, 0.2),
    (0.35, 0.2, 0.2),
    (0.30, 0.2, 0.2),
]

# Main loop - Pick and place each item
for i, (px, py, pz) in enumerate(pickup_poses):
    # PICKUP PHASE
    arm_1.set_start_state_to_current_state()
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "arm_1_base_link"
    pose_goal.pose.position.x = px
    pose_goal.pose.position.y = py
    pose_goal.pose.position.z = pz
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 0.7071
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.7071

    arm_1.set_goal_state(pose_stamped_msg=pose_goal, pose_link="arm_1_tool_link")
    plan_result = arm_1.plan()

    if plan_result:
        moveit_py.execute(plan_result.trajectory, controllers=[])
        print(f"Pickup {i + 1}: Moved to ({px}, {py}, {pz})")
    else:
        print(f"Pickup {i + 1}: Planning failed, continuing...")
        continue

    # Close gripper (PICKUP)
    tool_cmd = ToolCommand()
    tool_cmd.cmd_type = 2  # CLOSE_GRIPPER
    tool_cmd.tool_id = 11
    tool_cmd.max_torque_percentage = 80
    tool_cmd.hold_torque_percentage = 30
    goal = Tool.Goal()
    goal.cmd = tool_cmd
    action_client.send_goal_async(goal)
    print(f"Pickup {i + 1}: Close gripper")
    time.sleep(3)  # Wait 3 seconds for gripper to close and pick up item

    # DROP PHASE
    drop_z = 0.2 + (i * 0.05)
    arm_1.set_start_state_to_current_state()
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "arm_1_base_link"
    pose_goal.pose.position.x = 0.30
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = drop_z
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 0.7071
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.7071

    arm_1.set_goal_state(pose_stamped_msg=pose_goal, pose_link="arm_1_tool_link")
    plan_result = arm_1.plan()

    if plan_result:
        moveit_py.execute(plan_result.trajectory, controllers=[])
        print(f"Drop {i + 1}: Moved to drop location (z={drop_z})")
    else:
        print(f"Drop {i + 1}: Planning failed, continuing...")
        continue

    # Open gripper (DROP)
    tool_cmd = ToolCommand()
    tool_cmd.cmd_type = 1  # OPEN_GRIPPER
    tool_cmd.tool_id = 11
    tool_cmd.max_torque_percentage = 80
    tool_cmd.hold_torque_percentage = 30
    goal = Tool.Goal()
    goal.cmd = tool_cmd
    action_client.send_goal_async(goal)
    print(f"Drop {i + 1}: Open gripper")
    time.sleep(2)  # Wait 2 seconds for gripper to open and item to settle

rclpy.shutdown()
