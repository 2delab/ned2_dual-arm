#!/usr/bin/env python3
"""
MoveIt2 Dual-Arm Control with Custom Joint Angles
Usage: python test5_1.py [a1j1] [a1j2] [a1j3] [a1j4] [a1j5] [a1j6] [a2j1] [a2j2] [a2j3] [a2j4] [a2j5] [a2j6]
Example: python test5_1.py 30 0 0 0 0 0 45 0 0 0 0 0
"""

import sys
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from math import radians


def main():
    # Parse arguments
    angles = [0.0] * 12
    if len(sys.argv) > 1:
        if sys.argv[1] in ["-h", "--help"]:
            print(__doc__)
            return True
        try:
            angles = [
                float(sys.argv[i + 1]) if i + 1 < len(sys.argv) else 0.0
                for i in range(12)
            ]
        except ValueError:
            print("Error: All angles must be numeric (in degrees)")
            print(__doc__)
            return False

    rclpy.init()
    node = rclpy.create_node("moveit_test_dual_arm_custom")
    action_client = ActionClient(node, MoveGroup, "/move_action")

    print("\nWaiting for move_group action server...")
    if not action_client.wait_for_server(timeout_sec=10):
        print("✗ move_group action server not available!")
        rclpy.shutdown()
        return False

    print("✓ Connected to move_group action server\n")

    # Create goal
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = "dual"
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.1
    goal.request.max_acceleration_scaling_factor = 0.1
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    # Build constraints from angles
    constraints_list = []
    arm_names = ["arm_1", "arm_2"]

    for arm_idx, arm_name in enumerate(arm_names):
        for joint_idx in range(1, 7):
            constraint = JointConstraint()
            constraint.joint_name = f"{arm_name}_joint_{joint_idx}"
            constraint.weight = 1.0
            constraint.position = radians(angles[arm_idx * 6 + joint_idx - 1])
            constraint.tolerance_above = radians(5)
            constraint.tolerance_below = radians(5)
            constraints_list.append(constraint)

    goal.request.goal_constraints.append(
        Constraints(joint_constraints=constraints_list)
    )

    # Print target angles
    print(f"Target angles (degrees):")
    print(f"  Arm 1: {angles[0:6]}")
    print(f"  Arm 2: {angles[6:12]}\n")

    # Send goal
    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    goal_handle = future.result()

    if goal_handle is None or not goal_handle.accepted:
        print("✗ Goal rejected by move_group")
        rclpy.shutdown()
        return False

    print("✓ Goal accepted - executing...\n")

    # Wait for result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
    result_response = result_future.result()

    if result_response is None or result_response.result is None:
        print("✗ No result returned from move_group")
        rclpy.shutdown()
        return False

    error_code = result_response.result.error_code.val

    if error_code == 1:  # SUCCESS
        print("✓ SUCCESS!")
        rclpy.shutdown()
        return True
    else:
        error_messages = {
            -1: "PLANNING_FAILED",
            0: "NO_SOLUTION_FOUND",
            2: "TIMED_OUT",
            3: "START_STATE_IN_COLLISION",
            5: "GOAL_IN_COLLISION",
        }
        print(f"✗ Failed: {error_messages.get(error_code, f'ERROR {error_code}')}")
        rclpy.shutdown()
        return False


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
