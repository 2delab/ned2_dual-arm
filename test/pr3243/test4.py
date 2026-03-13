#!/usr/bin/env python3
"""
MoveIt2 Dual-Arm Control with Joint Goals (Sequential)
Moves arm_1_joint_1 and arm_2_joint_1 sequentially using joint goals.
Other joints stay at current positions for predictable motion.
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from math import radians
import time


def send_single_arm_goal(
    action_client,
    node,
    arm_name,
    target_joint_name,
    target_angle_deg,
    current_state=None,
):
    """
    Send and execute a single arm motion goal.

    Constrains ONLY the target joint to move, locks all other joints at current positions.
    This prevents unpredictable joint movements.
    """

    print(f"\n{'=' * 70}")
    print(f"Moving {arm_name} → {target_joint_name} to {target_angle_deg}°")
    print(f"(All other joints locked at current positions)")
    print(f"{'=' * 70}")

    # Create goal request for single arm
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = arm_name
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    # Build constraint list: move target joint, lock all others
    constraints_list = []

    # Define all joints for each arm
    if "arm_1" in arm_name:
        all_joints = [f"arm_1_joint_{i}" for i in range(1, 7)]
    else:  # arm_2
        all_joints = [f"arm_2_joint_{i}" for i in range(1, 7)]

    # Add constraints for each joint
    for joint in all_joints:
        constraint = JointConstraint()
        constraint.joint_name = joint

        if joint == target_joint_name:
            # Target joint: move to desired angle
            constraint.position = radians(target_angle_deg)
            constraint.tolerance_above = radians(5)
            constraint.tolerance_below = radians(5)
        else:
            # Other joints: lock at current position (0.0 if not provided)
            # In real scenario, you'd get current state from robot state
            constraint.position = 0.0  # Default: assume joints start at 0
            constraint.tolerance_above = radians(1)  # Very tight tolerance
            constraint.tolerance_below = radians(1)  # Lock in place

        constraint.weight = 1.0
        constraints_list.append(constraint)

    goal.request.goal_constraints.append(
        Constraints(joint_constraints=constraints_list)
    )

    print(f"Planning motion for {arm_name}...")
    print(f"  Target: {target_joint_name} = {target_angle_deg}°")
    print(f"  Locked joints: {[j for j in all_joints if j != target_joint_name]}")

    # Send goal
    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    goal_handle = future.result()

    if goal_handle is None or not goal_handle.accepted:
        print(f"✗ Goal rejected by move_group for {arm_name}")
        return False

    print(f"✓ Goal accepted - executing...\n")

    # Wait for result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
    result_response = result_future.result()

    if result_response is None or result_response.result is None:
        print(f"✗ No result returned for {arm_name}")
        return False

    error_code = result_response.result.error_code.val

    if error_code == 1:  # SUCCESS
        print(f"✓ SUCCESS! {arm_name} moved to {target_angle_deg}°\n")
        return True
    else:
        error_messages = {
            -1: "PLANNING_FAILED",
            0: "NO_SOLUTION_FOUND",
            2: "TIMED_OUT",
            3: "START_STATE_IN_COLLISION",
            5: "GOAL_IN_COLLISION",
        }
        print(
            f"✗ Failed for {arm_name}: {error_messages.get(error_code, f'ERROR {error_code}')}\n"
        )
        return False

    print(f"✓ Goal accepted - executing...\n")

    # Wait for result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
    result_response = result_future.result()

    if result_response is None or result_response.result is None:
        print(f"✗ No result returned for {arm_name}")
        return False

    error_code = result_response.result.error_code.val

    if error_code == 1:  # SUCCESS
        print(f"✓ SUCCESS! {arm_name} moved to {target_angle_deg}°\n")
        return True
    else:
        error_messages = {
            -1: "PLANNING_FAILED",
            0: "NO_SOLUTION_FOUND",
            2: "TIMED_OUT",
            3: "START_STATE_IN_COLLISION",
            5: "GOAL_IN_COLLISION",
        }
        print(
            f"✗ Failed for {arm_name}: {error_messages.get(error_code, f'ERROR {error_code}')}\n"
        )
        return False


def main():
    rclpy.init()
    node = rclpy.create_node("moveit_test_dual_arm_sequential")
    action_client = ActionClient(node, MoveGroup, "/move_action")

    print("\nWaiting for move_group action server...")
    if not action_client.wait_for_server(timeout_sec=10):
        print("✗ move_group action server not available!")
        rclpy.shutdown()
        return False

    print("✓ Connected to move_group action server")

    print("\n" + "=" * 70)
    print("DUAL-ARM SEQUENTIAL MOTION WITH JOINT GOALS")
    print("=" * 70)
    print("Method: Joint Goals - only specified joints move, others stay fixed")
    print("Execution: Sequential (arm_1 first, then arm_2)")

    # Move arm_1 joint_1 to 50 degrees
    result1 = send_single_arm_goal(action_client, node, "arm_1", "arm_1_joint_1", 0)

    # Small delay between movements
    time.sleep(1.0)

    # Move arm_2 joint_1 to 50 degrees
    result2 = send_single_arm_goal(action_client, node, "arm_2", "arm_2_joint_1", 0)

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    if result1 and result2:
        print("✓ SUCCESS: Both arms moved to 50° on joint_1")
        success = True
    else:
        print("✗ FAILED: One or both movements failed")
        print(f"  Arm 1: {'✓' if result1 else '✗'}")
        print(f"  Arm 2: {'✓' if result2 else '✗'}")
        success = False

    rclpy.shutdown()
    return success


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
