#!/usr/bin/env python3

import rclpy
import os
import time
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import Pose
from rclpy.action.client import ActionClient
from rclpy.node import Node
from niryo_ned_ros2_interfaces.action import Tool
from niryo_ned_ros2_interfaces.msg import ToolCommand

rclpy.init()

# Create node for action client
node = Node("demo4_node")

# Load config file
config_file = os.path.join(
    get_package_share_directory("niryo_ned2_dual_gripper_moveit_config"),
    "config",
    "moveit_py_params.yaml",
)

# Create MoveIt instance
moveit_py = MoveItPy(node_name="demo4_test", launch_params_filepaths=[config_file])

# Get planning component for dual arm
dual = moveit_py.get_planning_component("dual")
dual.set_start_state_to_current_state()

# Wait for robot state to be available
print("Waiting for robot state...")
time.sleep(2.0)

# Get robot model and create a fresh robot state
robot_model = moveit_py.get_robot_model()
goal_state = RobotState(robot_model)

# Get current state from planning scene monitor
planning_scene_monitor = moveit_py.get_planning_scene_monitor()
with planning_scene_monitor.read_only() as scene:
    current_state = scene.current_state
    # Copy joint values from current state
    goal_state.joint_positions = current_state.joint_positions

print("Phase 1: Setting up dual-arm IK for target poses...")

# Create Pose object for arm_1 (in arm_1_base_link frame)
pose_arm_1 = Pose()
pose_arm_1.position.x = 0.35
pose_arm_1.position.y = 0.2
pose_arm_1.position.z = 0.2
pose_arm_1.orientation.x = 0.0
pose_arm_1.orientation.y = 0.7071
pose_arm_1.orientation.z = 0.0
pose_arm_1.orientation.w = 0.7071

# Create Pose object for arm_2 (in arm_2_base_link frame)
pose_arm_2 = Pose()
pose_arm_2.position.x = 0.35
pose_arm_2.position.y = 0.2
pose_arm_2.position.z = 0.2
pose_arm_2.orientation.x = 0.0
pose_arm_2.orientation.y = 0.7071
pose_arm_2.orientation.z = 0.0
pose_arm_2.orientation.w = 0.7071

# Solve IK for arm_1
print("Phase 1: Solving IK for arm_1 at (0.35, 0.2, 0.2)...")
ik_success_arm_1 = goal_state.set_from_ik(
    joint_model_group_name="arm_1",
    geometry_pose=pose_arm_1,
    tip_name="arm_1_tool_link",
    timeout=5.0,
)

if not ik_success_arm_1:
    print("Phase 1: IK failed for arm_1 - target pose may be unreachable")
    rclpy.shutdown()
    exit(1)

print("Phase 1: IK solved successfully for arm_1")

# Solve IK for arm_2
print("Phase 1: Solving IK for arm_2 at (0.35, 0.2, 0.2)...")
ik_success_arm_2 = goal_state.set_from_ik(
    joint_model_group_name="arm_2",
    geometry_pose=pose_arm_2,
    tip_name="arm_2_tool_link",
    timeout=5.0,
)

if not ik_success_arm_2:
    print("Phase 1: IK failed for arm_2 - target pose may be unreachable")
    rclpy.shutdown()
    exit(1)

print("Phase 1: IK solved successfully for arm_2")
print("Phase 1: Both arms IK solved successfully - planning coordinated motion...")

# Set the dual-arm goal state
dual.set_goal_state(robot_state=goal_state)

# Plan the dual-arm motion
plan_result = dual.plan()

if plan_result:
    print("Phase 1: Planning succeeded - executing coordinated dual-arm motion...")
    moveit_py.execute(plan_result.trajectory, controllers=[])
    print("Phase 1: Successfully moved both arms to target poses (0.35, 0.2, 0.2)")
else:
    print("Phase 1: Planning failed - no valid trajectory found")

# Proper shutdown sequence
print("\nShutting down gracefully...")
rclpy.shutdown()
