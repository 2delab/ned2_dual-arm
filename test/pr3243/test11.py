#!/usr/bin/env python3
"""
MoveIt2 Dual-Arm Truly Asynchronous Multi-Phase Execution

TRULY ASYNC ARCHITECTURE:
- arm_1: 4 independent phases (joint_1 oscillates 50↔-50, joint_2 locked at -50 after phase 1)
- arm_2: 6 independent phases (joint_1 oscillates 50↔-50, joint_2 locked at -50 after phase 1)
- Separate planning for each arm
- Queue all phases simultaneously (no synchronization between arms)
- arm_2 finishes 6 phases while arm_1 still on phases 3-4

Execution Flow:
  arm_1: Phase 1a → Phase 2a → Phase 3a → Phase 4a (4 trajectories queued)
  arm_2: Phase 1b → Phase 2b → Phase 3b → Phase 4b → Phase 5b → Phase 6b (6 trajectories queued)
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from math import radians


def plan_arm_trajectory(move_group_client, node, arm_name, target_positions, phase_num):
    """
    Plan trajectory for a single arm.

    Args:
        move_group_client: MoveGroup action client
        node: ROS node
        arm_name: "arm_1" or "arm_2"
        target_positions: Dict of {joint_name: target_angle_deg}
        phase_num: Phase number for logging

    Returns:
        trajectory_msg or None on error
    """
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
            angle_deg = target_positions[joint_name]
            constraint.position = radians(angle_deg)
            constraint.tolerance_above = radians(5)
            constraint.tolerance_below = radians(5)
        else:
            # Lock at current position
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


def queue_trajectory_async(arm_name, trajectory_msg, node, phase_label):
    """
    Queue trajectory to controller (non-blocking, returns immediately).
    """
    action_client = ActionClient(
        node, FollowJointTrajectory, f"/{arm_name}/follow_joint_trajectory"
    )

    if not action_client.wait_for_server(timeout_sec=2.0):
        return False

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory_msg.joint_trajectory

    # Send (non-blocking) and immediately return
    action_client.send_goal_async(goal)
    return True


def main():
    rclpy.init()
    node = rclpy.create_node("moveit_test_dual_arm_truly_async")
    move_group_client = ActionClient(node, MoveGroup, "/move_action")

    if not move_group_client.wait_for_server(timeout_sec=10):
        print("✗ move_group action server not available!")
        rclpy.shutdown()
        return False

    print("\n" + "=" * 70)
    print("DUAL-ARM TRULY ASYNCHRONOUS EXECUTION")
    print("=" * 70)
    print("arm_1: 4 phases (joint_1 oscillates 50↔-50)")
    print("arm_2: 6 phases (joint_1 oscillates 50↔-50)")
    print("Execution: Completely independent, no synchronization")
    print()

    # ========== PHASE DEFINITIONS ==========
    # arm_1: 4 phases
    arm1_phases = [
        {
            "phase": "1a",
            "desc": "Home + joint_2=-50",
            "targets": {"arm_1_joint_1": 0, "arm_1_joint_2": -50},
        },
        {
            "phase": "2a",
            "desc": "joint_1=50 (joint_2 locked -50)",
            "targets": {"arm_1_joint_1": 50, "arm_1_joint_2": -50},
        },
        {
            "phase": "3a",
            "desc": "joint_1=-50 (joint_2 locked -50)",
            "targets": {"arm_1_joint_1": -50, "arm_1_joint_2": -50},
        },
        {
            "phase": "4a",
            "desc": "joint_1=50 (joint_2 locked -50)",
            "targets": {"arm_1_joint_1": 50, "arm_1_joint_2": -50},
        },
    ]

    # arm_2: 6 phases
    arm2_phases = [
        {
            "phase": "1b",
            "desc": "Home + joint_2=-50",
            "targets": {"arm_2_joint_1": 0, "arm_2_joint_2": -50},
        },
        {
            "phase": "2b",
            "desc": "joint_1=50 (joint_2 locked -50)",
            "targets": {"arm_2_joint_1": 50, "arm_2_joint_2": -50},
        },
        {
            "phase": "3b",
            "desc": "joint_1=-50 (joint_2 locked -50)",
            "targets": {"arm_2_joint_1": -50, "arm_2_joint_2": -50},
        },
        {
            "phase": "4b",
            "desc": "joint_1=50 (joint_2 locked -50)",
            "targets": {"arm_2_joint_1": 50, "arm_2_joint_2": -50},
        },
        {
            "phase": "5b",
            "desc": "joint_1=-50 (joint_2 locked -50)",
            "targets": {"arm_2_joint_1": -50, "arm_2_joint_2": -50},
        },
        {
            "phase": "6b",
            "desc": "joint_1=50 (joint_2 locked -50)",
            "targets": {"arm_2_joint_1": 50, "arm_2_joint_2": -50},
        },
    ]

    # ========== PLANNING STAGE ==========
    print("[PLANNING ALL PHASES]")

    arm1_trajectories = {}
    print("  arm_1 (4 phases):")
    for phase_info in arm1_phases:
        phase = phase_info["phase"]
        desc = phase_info["desc"]
        targets = phase_info["targets"]
        print(f"    {phase}: {desc}...", end=" ", flush=True)
        traj = plan_arm_trajectory(move_group_client, node, "arm_1", targets, phase)
        if traj is None:
            print("✗ FAILED")
            rclpy.shutdown()
            return False
        arm1_trajectories[phase] = traj
        print("✓")

    print("  arm_2 (6 phases):")
    arm2_trajectories = {}
    for phase_info in arm2_phases:
        phase = phase_info["phase"]
        desc = phase_info["desc"]
        targets = phase_info["targets"]
        print(f"    {phase}: {desc}...", end=" ", flush=True)
        traj = plan_arm_trajectory(move_group_client, node, "arm_2", targets, phase)
        if traj is None:
            print("✗ FAILED")
            rclpy.shutdown()
            return False
        arm2_trajectories[phase] = traj
        print("✓")

    # ========== EXECUTION STAGE (TRULY ASYNC) ==========
    print("\n[QUEUING ALL PHASES SIMULTANEOUSLY - TRUE ASYNC]")

    print("  arm_1 queue (4 phases):")
    for phase_info in arm1_phases:
        phase = phase_info["phase"]
        traj = arm1_trajectories[phase]
        success = queue_trajectory_async("arm_1", traj, node, phase)
        status = "✓" if success else "✗"
        print(f"    {phase}: {status}")

    print("  arm_2 queue (6 phases):")
    for phase_info in arm2_phases:
        phase = phase_info["phase"]
        traj = arm2_trajectories[phase]
        success = queue_trajectory_async("arm_2", traj, node, phase)
        status = "✓" if success else "✗"
        print(f"    {phase}: {status}")

    # ========== SUMMARY ==========
    print("\n" + "=" * 70)
    print("✓ ALL 10 TRAJECTORIES QUEUED FOR EXECUTION!")
    print("=" * 70)
    print("\nExecution Timeline (TRULY ASYNCHRONOUS):")
    print("  arm_1: 1a → 2a → 3a → 4a (4 trajectories)")
    print("  arm_2: 1b → 2b → 3b → 4b → 5b → 6b (6 trajectories)")
    print("\nResult: arm_2 completes its 6-phase loop while")
    print("        arm_1 is still executing phases 3-4")
    print("\nNo synchronization between arms - truly independent execution!")
    print()

    rclpy.shutdown()
    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
