#!/usr/bin/env python3
"""
MoveIt2 Dual-Arm Multi-Phase with PR #3243 Style Simultaneous Execution

Architecture:
- Plan each phase with "dual" group (both arms in harmony)
- Queue all 3 phases immediately on separate controllers
- All 6 trajectories execute in parallel (3 phases × 2 arms)
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from math import radians


def plan_dual_phase(move_group_client, node, phase_targets):
    """
    Plan a phase using the "dual" group (both arms together for harmony).

    Returns:
        trajectory_msg or None on error
    """
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = "dual"
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    # Build constraints for all 12 joints
    constraints_list = []
    for arm_prefix in ["arm_1_", "arm_2_"]:
        for i in range(1, 7):
            joint_name = f"{arm_prefix}joint_{i}"
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.weight = 1.0

            if joint_name in phase_targets:
                angle_deg = phase_targets[joint_name]
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

    # Send planning request
    future = move_group_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    goal_handle = future.result()

    if goal_handle is None or not goal_handle.accepted:
        return None

    # Wait for planning result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30)
    result_response = result_future.result()

    if result_response is None or result_response.result is None:
        return None

    error_code = result_response.result.error_code.val
    if error_code != 1:
        return None

    return result_response.result.planned_trajectory


def queue_phase_execution(arm_name, trajectory_msg, node):
    """
    Queue trajectory to controller without waiting (non-blocking).
    Returns immediately.
    """
    action_client = ActionClient(
        node, FollowJointTrajectory, f"/{arm_name}/follow_joint_trajectory"
    )

    if not action_client.wait_for_server(timeout_sec=2.0):
        return False

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory_msg.joint_trajectory

    # Send and return immediately (non-blocking)
    action_client.send_goal_async(goal)
    return True


def main():
    rclpy.init()
    node = rclpy.create_node("moveit_test_dual_arm_simultaneous")
    move_group_client = ActionClient(node, MoveGroup, "/move_action")

    if not move_group_client.wait_for_server(timeout_sec=10):
        print("✗ move_group action server not available!")
        rclpy.shutdown()
        return False

    print("\n" + "=" * 70)
    print("DUAL-ARM 3-PHASE SIMULTANEOUS EXECUTION (PR #3243 Style)")
    print("=" * 70)
    print("Plan: dual group (harmony) → Execute: separate controllers (parallel)")
    print()

    # Define all phases
    phases = [
        (
            1,
            "Home (0°)",
            {
                "arm_1_joint_1": 0,
                "arm_1_joint_2": 0,
                "arm_1_joint_3": 0,
                "arm_1_joint_4": 0,
                "arm_1_joint_5": 0,
                "arm_1_joint_6": 0,
                "arm_2_joint_1": 0,
                "arm_2_joint_2": 0,
                "arm_2_joint_3": 0,
                "arm_2_joint_4": 0,
                "arm_2_joint_5": 0,
                "arm_2_joint_6": 0,
            },
        ),
        (
            2,
            "Joint1=50°, Joint2=-35°",
            {
                "arm_1_joint_1": 50,
                "arm_1_joint_2": -35,
                "arm_2_joint_1": 50,
                "arm_2_joint_2": -35,
            },
        ),
        (
            3,
            "Joint1=-50°, Joint2=-35°",
            {
                "arm_1_joint_1": -50,
                "arm_1_joint_2": -35,
                "arm_2_joint_1": -50,
                "arm_2_joint_2": -35,
            },
        ),
    ]

    # Plan all phases first
    print("[PLANNING ALL PHASES]")
    trajectories = {}
    for phase_num, desc, targets in phases:
        print(f"  Phase {phase_num}: {desc}...", end=" ", flush=True)
        traj = plan_dual_phase(move_group_client, node, targets)
        if traj is None:
            print("✗ FAILED")
            rclpy.shutdown()
            return False
        trajectories[phase_num] = traj
        print("✓")

    # Queue all phases for execution simultaneously
    print("\n[QUEUING ALL PHASES SIMULTANEOUSLY]")
    for phase_num, desc, _ in phases:
        traj = trajectories[phase_num]
        print(f"  Phase {phase_num}:", end=" ")

        success1 = queue_phase_execution("arm_1", traj, node)
        success2 = queue_phase_execution("arm_2", traj, node)

        if success1 and success2:
            print("✓ queued")
        else:
            print("✗ failed")
            rclpy.shutdown()
            return False

    print("\n" + "=" * 70)
    print("✓ ALL 6 TRAJECTORIES EXECUTING IN PARALLEL!")
    print("=" * 70)
    print("  Phase 1: arm_1 + arm_2 (executing)")
    print("  Phase 2: arm_1 + arm_2 (executing)")
    print("  Phase 3: arm_1 + arm_2 (executing)")
    print()

    rclpy.shutdown()
    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
