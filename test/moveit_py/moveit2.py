#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from math import radians
rclpy.init()
node = rclpy.create_node("moveit_simple")
action_client = ActionClient(node, MoveGroup, "/move_action")
if not action_client.wait_for_server(timeout_sec=5):
    print("✗ Server not available")
    rclpy.shutdown()
    exit(1)
# Create goal
goal = MoveGroup.Goal()
goal.request = MotionPlanRequest()
goal.request.group_name = "arm_1"
goal.request.max_velocity_scaling_factor = 0.1
goal.request.num_planning_attempts = 10
goal.request.allowed_planning_time = 5.0
# Constraints: joint_1 to 50°, others locked at 0°
constraints_list = []
for i in range(1, 7):
    constraint = JointConstraint()
    constraint.joint_name = f"arm_1_joint_{i}"
    constraint.position = radians(50 if i == 1 else 0)
    constraint.tolerance_above = radians(5 if i == 1 else 1)
    constraint.tolerance_below = radians(5 if i == 1 else 1)
    constraint.weight = 1.0
    constraints_list.append(constraint)
goal.request.goal_constraints.append(Constraints(joint_constraints=constraints_list))
# Send and execute
future = action_client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future, timeout_sec=5)
goal_handle = future.result()
if goal_handle and goal_handle.accepted:
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
    result = result_future.result()
    if result and result.result.error_code.val == 1:
        print("✓ Success!")
    else:
        print("✗ Failed")
else:
    print("✗ Goal rejected")
rclpy.shutdown()