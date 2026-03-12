#!/usr/bin/env python3
"""
MoveIt2 Dual-Arm Multi-Phase Control Sequence with True Async Parallel Execution

TRUE ASYNC PATTERN: No waiting between phases
- Phase 1: Plan both arms → Send to controllers → Return immediately
- Phase 2: Plan both arms → Send to controllers → Return immediately
- Phase 3: Plan both arms → Send to controllers → Return immediately

All 6 trajectories (3 phases × 2 arms) execute in parallel on separate controllers!
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from math import radians
import time


def plan_arm_trajectory(move_group_client, node, arm_name, target_positions):
    """
    Plan trajectory for a single arm using MoveGroup.

    Args:
        move_group_client: MoveGroup action client
        node: ROS node
        arm_name: "arm_1" or "arm_2"
        target_positions: Dict of {joint_name: target_angle_deg}

    Returns:
        trajectory_msg if successful, None otherwise
    """

    # Create goal request for single arm
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = arm_name
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    # Build constraints for this arm only
    constraints_list = []
    arm_prefix = f"{arm_name}_"

    for i in range(1, 7):
        joint_name = f"{arm_prefix}joint_{i}"
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.weight = 1.0

        if joint_name in target_positions:
            # This joint is part of the target
            angle_deg = target_positions[joint_name]
            constraint.position = radians(angle_deg)
            constraint.tolerance_above = radians(5)
            constraint.tolerance_below = radians(5)
        else:
            # This joint should stay locked
            constraint.position = 0.0
            constraint.tolerance_above = radians(1)  # Very tight
            constraint.tolerance_below = radians(1)  # Lock in place

        constraints_list.append(constraint)

    goal.request.goal_constraints.append(
        Constraints(joint_constraints=constraints_list)
    )

    # Send planning goal
    future = move_group_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    goal_handle = future.result()

    if goal_handle is None or not goal_handle.accepted:
        print(f"      ✗ Planning rejected for {arm_name}")
        return None

    # Wait for planning result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
    result_response = result_future.result()

    if result_response is None or result_response.result is None:
        print(f"      ✗ No planning result for {arm_name}")
        return None

    error_code = result_response.result.error_code.val
    if error_code != 1:  # SUCCESS
        print(f"      ✗ Planning failed for {arm_name} (error {error_code})")
        return None

    print(
        f"      ✓ {arm_name} planned ({len(result_response.result.planned_trajectory.joint_trajectory.points)} points)"
    )
    return result_response.result.planned_trajectory


def send_to_controller_async(arm_name, trajectory_msg, node):
    """
    Send trajectory to arm controller (NON-BLOCKING, returns immediately).

    Args:
        arm_name: "arm_1" or "arm_2"
        trajectory_msg: RobotTrajectory message
        node: ROS node

    Returns:
        True if sent successfully (doesn't wait for execution)
    """

    # Create action client for this arm's controller
    action_client = ActionClient(
        node, FollowJointTrajectory, f"/{arm_name}/follow_joint_trajectory"
    )

    # Wait for server
    if not action_client.wait_for_server(timeout_sec=2.0):
        print(f"      ✗ Controller not available for {arm_name}")
        return False

    # Create execution goal
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory_msg.joint_trajectory

    # Send goal (non-blocking) - this returns immediately!
    future = action_client.send_goal_async(goal)

    # Don't wait for result - just send and move on
    print(f"      ✓ {arm_name} sent to controller (execution in progress...)")
    return True


def execute_phase_async(
    move_group_client, node, phase_num, phase_description, targets_arm1, targets_arm2
):
    """
    Execute a phase ASYNCHRONOUSLY: plan for both arms, then send to controllers.
    Returns immediately without waiting for execution.

    Args:
        move_group_client: MoveGroup action client for planning
        node: ROS node
        phase_num: Phase number
        phase_description: Description
        targets_arm1: Target positions for arm_1
        targets_arm2: Target positions for arm_2

    Returns:
        True if successfully sent to controllers (doesn't guarantee execution success)
    """

    print(f"\n{'=' * 70}")
    print(f"PHASE {phase_num}: {phase_description}")
    print(f"{'=' * 70}")

    # ========== PLANNING STAGE ==========
    print(f"\n  [PLANNING]")

    traj1 = plan_arm_trajectory(move_group_client, node, "arm_1", targets_arm1)
    if traj1 is None:
        print(f"  ✗ Planning failed for arm_1")
        return False

    traj2 = plan_arm_trajectory(move_group_client, node, "arm_2", targets_arm2)
    if traj2 is None:
        print(f"  ✗ Planning failed for arm_2")
        return False

    # ========== EXECUTION STAGE (SEND NON-BLOCKING) ==========
    print(f"\n  [SENDING TO CONTROLLERS - ASYNC]")

    # Send both trajectories (non-blocking, returns immediately)
    success1 = send_to_controller_async("arm_1", traj1, node)
    success2 = send_to_controller_async("arm_2", traj2, node)

    if not (success1 and success2):
        print(f"  ✗ Failed to send trajectories")
        return False

    print(f"\n  ✓ Phase {phase_num} SENT (both arms executing in parallel!)")
    return True


def main():
    rclpy.init()
    node = rclpy.create_node("moveit_test_dual_arm_async_phases")
    move_group_client = ActionClient(node, MoveGroup, "/move_action")

    print("\nWaiting for move_group action server...")
    if not move_group_client.wait_for_server(timeout_sec=10):
        print("✗ move_group action server not available!")
        rclpy.shutdown()
        return False

    print("✓ Connected to move_group action server")

    print("\n" + "=" * 70)
    print("DUAL-ARM 3-PHASE ASYNC PARALLEL EXECUTION")
    print("=" * 70)
    print("TRUE ASYNC: Plans all phases, sends all to controllers immediately")
    print("No waiting between phases - all execute in parallel!")
    print("\nPhase 1: Home position (all joints to 0°)")
    print("Phase 2: Move joint_1 to 50° and joint_2 to -35°")
    print("Phase 3: Move joint_1 to -50° (keep joint_2 at -35°)")

    # ========== SEND ALL PHASES (NO WAITING) ==========

    # Phase 1: Move all joints to 0
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
    result1 = execute_phase_async(
        move_group_client,
        node,
        1,
        "Home Position (All joints to 0°)",
        phase1_arm1,
        phase1_arm2,
    )

    # Phase 2: Move joint_1 to 50° and joint_2 to -35°
    phase2_arm1 = {"arm_1_joint_1": 50, "arm_1_joint_2": -35}
    phase2_arm2 = {"arm_2_joint_1": 50, "arm_2_joint_2": -35}
    result2 = execute_phase_async(
        move_group_client,
        node,
        2,
        "Move joint_1 to 50° and joint_2 to -35°",
        phase2_arm1,
        phase2_arm2,
    )

    # Phase 3: Move joint_1 to -50° (keep joint_2 at -35°)
    phase3_arm1 = {"arm_1_joint_1": -50, "arm_1_joint_2": -35}
    phase3_arm2 = {"arm_2_joint_1": -50, "arm_2_joint_2": -35}
    result3 = execute_phase_async(
        move_group_client,
        node,
        3,
        "Move joint_1 to -50° (keep joint_2 at -35°)",
        phase3_arm1,
        phase3_arm2,
    )

    # Summary
    print("\n" + "=" * 70)
    print("EXECUTION SUMMARY")
    print("=" * 70)
    if result1 and result2 and result3:
        print("✓ ALL PHASES QUEUED FOR EXECUTION!")
        print("\nStatus: 6 trajectories now executing in parallel:")
        print("  - Phase 1: arm_1 + arm_2 (in parallel)")
        print("  - Phase 2: arm_1 + arm_2 (in parallel)")
        print("  - Phase 3: arm_1 + arm_2 (in parallel)")
        print(
            "\nNote: Script exits immediately. Controllers handle execution asynchronously."
        )
        success = True
    else:
        print("✗ SOME PHASES FAILED TO QUEUE")
        print(f"  Phase 1: {'✓' if result1 else '✗'}")
        print(f"  Phase 2: {'✓' if result2 else '✗'}")
        print(f"  Phase 3: {'✓' if result3 else '✗'}")
        success = False

    rclpy.shutdown()
    return success


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
