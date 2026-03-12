#!/usr/bin/env python3
"""
MoveIt2 Dual-Arm Control with Joint Constraints (Simultaneous)
Moves both arm_1_joint_1 and arm_2_joint_1 to 50 degrees simultaneously.
All other joints are locked at current positions for predictable motion.
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from math import radians


def main():
    rclpy.init()
    node = rclpy.create_node("moveit_test_dual_arm")
    action_client = ActionClient(node, MoveGroup, "/move_action")

    print("\nWaiting for move_group action server...")
    if not action_client.wait_for_server(timeout_sec=10):
        print("✗ move_group action server not available!")
        rclpy.shutdown()
        return False

    print("✓ Connected to move_group action server\n")

    # Create goal request
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = "dual"
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    # Build constraint list: move only joint_1 on each arm, lock all others
    constraints_list = []

    # Arm 1 constraints
    for i in range(1, 7):
        constraint = JointConstraint()
        constraint.joint_name = f"arm_1_joint_{i}"
        constraint.weight = 1.0

        if i == 1:
            # Joint 1: move to 50 degrees
            constraint.position = radians(50)
            constraint.tolerance_above = radians(5)
            constraint.tolerance_below = radians(5)
        else:
            # Other joints: lock at current position (0.0)
            constraint.position = 0.0
            constraint.tolerance_above = radians(1)  # Very tight
            constraint.tolerance_below = radians(1)  # Lock in place

        constraints_list.append(constraint)

    # Arm 2 constraints
    for i in range(1, 7):
        constraint = JointConstraint()
        constraint.joint_name = f"arm_2_joint_{i}"
        constraint.weight = 1.0

        if i == 1:
            # Joint 1: move to 50 degrees
            constraint.position = radians(50)
            constraint.tolerance_above = radians(5)
            constraint.tolerance_below = radians(5)
        else:
            # Other joints: lock at current position (0.0)
            constraint.position = 0.0
            constraint.tolerance_above = radians(1)  # Very tight
            constraint.tolerance_below = radians(1)  # Lock in place

        constraints_list.append(constraint)

    goal.request.goal_constraints.append(
        Constraints(joint_constraints=constraints_list)
    )

    print("Planning dual-arm motion to 50° on both joint_1...")
    print(f"  Group: {goal.request.group_name}")
    print(f"  Planner: {goal.request.planner_id}")
    print(f"  Max velocity: {goal.request.max_velocity_scaling_factor}")
    print(f"  Motion: SIMULTANEOUS (both arms move at same time)")
    print(f"  Constraints: 12 joints total")
    print(f"    - arm_1_joint_1 & arm_2_joint_1 → move to 50°")
    print(f"    - all other joints (2-6 on each arm) → locked at 0°")
    print()

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
        print("Both arms moved to 50° on joint_1")
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
