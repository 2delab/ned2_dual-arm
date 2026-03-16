#!/usr/bin/env python3
import rclpy
from rclpy.action.client import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
)
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header

rclpy.init()
node = rclpy.create_node("moveit_pose_goal")
action_client = ActionClient(node, MoveGroup, "/move_action")

if not action_client.wait_for_server(timeout_sec=5):
    print("✗ Server not available")
    rclpy.shutdown()
    exit(1)

# Create goal with pose specification
goal = MoveGroup.Goal()
goal.request = MotionPlanRequest()
goal.request.group_name = "arm_1"
goal.request.max_velocity_scaling_factor = 0.1
goal.request.num_planning_attempts = 10
goal.request.allowed_planning_time = 5.0

# Position constraint: x=0.3, y=0.0, z=0.2
pc = PositionConstraint()
pc.header = Header(frame_id="base_link")
pc.link_name = "arm_1_tool_link"
pc.target_point_offset = Vector3(x=0.1, y=0.0, z=0.2)
pc.weight = 1.0

# Orientation constraint: roll=0.0, pitch=1.56, yaw=0.0
# Quaternion: [x, y, z, w] = [0.0, 0.703279, 0.0, 0.710914]
oc = OrientationConstraint()
oc.header = Header(frame_id="base_link")
oc.link_name = "arm_1_tool_link"
oc.orientation = Quaternion(x=0.0, y=0.703279, z=0.0, w=0.710914)
oc.absolute_x_axis_tolerance = 0.1
oc.absolute_y_axis_tolerance = 0.1
oc.absolute_z_axis_tolerance = 0.1
oc.weight = 1.0

goal.request.goal_constraints.append(
    Constraints(position_constraints=[pc], orientation_constraints=[oc])
)

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
