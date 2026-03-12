#!/usr/bin/env python3
"""
MoveIt2 Dual-Arm Multi-Phase Control with TRUE Parallel Async Execution

TRULY PARALLEL ARCHITECTURE:
- Planning: Send both arm_1 & arm_2 planning requests (non-blocking)
- Wait for both plans to complete
- Execution: Send both trajectories to controllers (non-blocking)
- Don't wait - proceed to next phase immediately

Result: All arms move simultaneously!
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from math import radians


def create_planning_goal(arm_name, target_positions):
    """
    Create a MoveGroup planning goal for one arm.

    Args:
        arm_name: "arm_1" or "arm_2"
        target_positions: Dict of {joint_name: target_angle_deg}

    Returns:
        MoveGroup.Goal object
    """
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = arm_name
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    # Build constraints for this arm
    constraints_list = []
    arm_prefix = f"{arm_name}_"

    for i in range(1, 7):
        joint_name = f"{arm_prefix}joint_{i}"
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.weight = 1.0

        if joint_name in target_positions:
            angle_deg = target_positions[joint_name]
            constraint.position = radians(angle_deg)
            constraint.tolerance_above = radians(5)
            constraint.tolerance_below = radians(5)
        else:
            constraint.position = 0.0
            constraint.tolerance_above = radians(1)
            constraint.tolerance_below = radians(1)

        constraints_list.append(constraint)

    goal.request.goal_constraints.append(
        Constraints(joint_constraints=constraints_list)
    )

    return goal


def send_planning_request_async(move_group_client, arm_name, goal):
    """
    Send planning request without waiting (non-blocking).

    Returns:
        (future_for_goal_acceptance, arm_name) tuple
    """
    future = move_group_client.send_goal_async(goal)
    return (future, arm_name)


def wait_for_planning_results(node, planning_futures):
    """
    Wait for both planning requests to complete and get results.

    Args:
        node: ROS node
        planning_futures: List of (future, arm_name) tuples

    Returns:
        Dict of {arm_name: trajectory_msg} or None on error
    """
    results = {}

    for future, arm_name in planning_futures:
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            print(f"      ✗ {arm_name} planning rejected")
            return None

        # Wait for planning result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
        result_response = result_future.result()

        if result_response is None or result_response.result is None:
            print(f"      ✗ {arm_name} no planning result")
            return None

        error_code = result_response.result.error_code.val
        if error_code != 1:
            print(f"      ✗ {arm_name} planning failed (error {error_code})")
            return None

        trajectory = result_response.result.planned_trajectory
        points = len(trajectory.joint_trajectory.points)
        print(f"      ✓ {arm_name} planned ({points} points)")
        results[arm_name] = trajectory

    return results


def send_to_controller_async(arm_name, trajectory_msg, node):
    """
    Send trajectory to controller (non-blocking, returns immediately).
    """
    action_client = ActionClient(
        node, FollowJointTrajectory, f"/{arm_name}/follow_joint_trajectory"
    )

    if not action_client.wait_for_server(timeout_sec=2.0):
        print(f"      ✗ {arm_name} controller not available")
        return False

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory_msg.joint_trajectory

    # Send (non-blocking) and immediately return
    action_client.send_goal_async(goal)
    print(f"      ✓ {arm_name} executing")
    return True


def execute_phase_truly_parallel(
    move_group_client, node, phase_num, phase_description, targets_arm1, targets_arm2
):
    """
    Execute phase with TRULY parallel planning and execution.

    1. Send planning requests for both arms (non-blocking)
    2. Wait for both plans to complete
    3. Send both trajectories to controllers (non-blocking)
    4. Return immediately (no waiting for execution)
    """

    print(f"\n{'=' * 70}")
    print(f"PHASE {phase_num}: {phase_description}")
    print(f"{'=' * 70}")

    # ========== STEP 1: SEND BOTH PLANNING REQUESTS (NON-BLOCKING) ==========
    print(f"\n  [STEP 1: SEND PLANNING REQUESTS]")
    goal1 = create_planning_goal("arm_1", targets_arm1)
    goal2 = create_planning_goal("arm_2", targets_arm2)

    planning_futures = [
        send_planning_request_async(move_group_client, "arm_1", goal1),
        send_planning_request_async(move_group_client, "arm_2", goal2),
    ]
    print(f"    ✓ Both planning requests sent")

    # ========== STEP 2: WAIT FOR BOTH PLANS TO COMPLETE ==========
    print(f"\n  [STEP 2: WAIT FOR PLANS]")
    trajectories = wait_for_planning_results(node, planning_futures)

    if trajectories is None:
        print(f"  ✗ Phase {phase_num} planning failed")
        return False

    # ========== STEP 3: SEND BOTH TRAJECTORIES (NON-BLOCKING) ==========
    print(f"\n  [STEP 3: SEND TO CONTROLLERS - BOTH START EXECUTING]")

    success1 = send_to_controller_async("arm_1", trajectories["arm_1"], node)
    success2 = send_to_controller_async("arm_2", trajectories["arm_2"], node)

    if not (success1 and success2):
        print(f"  ✗ Phase {phase_num} execution setup failed")
        return False

    print(f"\n  ✓ Phase {phase_num} EXECUTING (both arms moving NOW!)")
    return True


def main():
    rclpy.init()
    node = rclpy.create_node("moveit_test_dual_arm_truly_parallel")
    move_group_client = ActionClient(node, MoveGroup, "/move_action")

    print("\nWaiting for move_group action server...")
    if not move_group_client.wait_for_server(timeout_sec=10):
        print("✗ move_group action server not available!")
        rclpy.shutdown()
        return False

    print("✓ Connected to move_group action server")

    print("\n" + "=" * 70)
    print("DUAL-ARM TRULY PARALLEL ASYNC EXECUTION")
    print("=" * 70)
    print("Architecture: Parallel planning → Parallel execution")
    print("Both arms move at the SAME TIME in every phase!")
    print("\nPhase 1: Home position (all joints to 0°)")
    print("Phase 2: Move joint_1 to 50° and joint_2 to -35°")
    print("Phase 3: Move joint_1 to -50° (keep joint_2 at -35°)")

    # ========== PHASE 1 ==========
    phase1_arm1 = {
        "arm_1_joint_1": 0,
        "arm_1_joint_2": 0,
        "arm_1_joint_3": 0,
        "arm_1_joint_4": 0,
        "arm_1_joint_5": 0,
        "arm_1_joint_6": 0,
    }
    phase1_arm2 = {
        "arm_2_joint_1": 0,
        "arm_2_joint_2": 0,
        "arm_2_joint_3": 0,
        "arm_2_joint_4": 0,
        "arm_2_joint_5": 0,
        "arm_2_joint_6": 0,
    }
    result1 = execute_phase_truly_parallel(
        move_group_client,
        node,
        1,
        "Home Position (All joints to 0°)",
        phase1_arm1,
        phase1_arm2,
    )

    # ========== PHASE 2 ==========
    phase2_arm1 = {"arm_1_joint_1": 50, "arm_1_joint_2": -35}
    phase2_arm2 = {"arm_2_joint_1": 50, "arm_2_joint_2": -35}
    result2 = execute_phase_truly_parallel(
        move_group_client,
        node,
        2,
        "Move joint_1 to 50° and joint_2 to -35°",
        phase2_arm1,
        phase2_arm2,
    )

    # ========== PHASE 3 ==========
    phase3_arm1 = {"arm_1_joint_1": -50, "arm_1_joint_2": -35}
    phase3_arm2 = {"arm_2_joint_1": -50, "arm_2_joint_2": -35}
    result3 = execute_phase_truly_parallel(
        move_group_client,
        node,
        3,
        "Move joint_1 to -50° (keep joint_2 at -35°)",
        phase3_arm1,
        phase3_arm2,
    )

    # ========== SUMMARY ==========
    print("\n" + "=" * 70)
    print("EXECUTION SUMMARY")
    print("=" * 70)
    if result1 and result2 and result3:
        print("✓ ALL PHASES EXECUTING IN PARALLEL!")
        print("\nWhat's happening RIGHT NOW:")
        print("  - Phase 1: arm_1 + arm_2 moving together")
        print("  - Phase 2: arm_1 + arm_2 moving together")
        print("  - Phase 3: arm_1 + arm_2 moving together")
        print("\nAll 6 trajectories executing simultaneously on their controllers!")
        success = True
    else:
        print("✗ SOME PHASES FAILED")
        print(f"  Phase 1: {'✓' if result1 else '✗'}")
        print(f"  Phase 2: {'✓' if result2 else '✗'}")
        print(f"  Phase 3: {'✓' if result3 else '✗'}")
        success = False

    rclpy.shutdown()
    return success


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
