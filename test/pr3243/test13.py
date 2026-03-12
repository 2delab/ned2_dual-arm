#!/usr/bin/env python3
"""
MoveIt2 Dual-Arm 3-Phase - Option A: Plan ALL in Parallel, Then Execute ALL in Parallel

ARCHITECTURE:
1. PLANNING PHASE (ALL 6 requests sent simultaneously):
   - Send Phase 1 arm_1 + Phase 1 arm_2 (non-blocking)
   - Send Phase 2 arm_1 + Phase 2 arm_2 (non-blocking)
   - Send Phase 3 arm_1 + Phase 3 arm_2 (non-blocking)
   - Then wait for ALL 6 results

2. EXECUTION PHASE (ALL 6 queued simultaneously):
   - Queue Phase 1 arm_1 + Phase 1 arm_2 (non-blocking)
   - Queue Phase 2 arm_1 + Phase 2 arm_2 (non-blocking)
   - Queue Phase 3 arm_1 + Phase 3 arm_2 (non-blocking)

Result: All 3 phases executing simultaneously on separate controllers
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from math import radians


def create_planning_goal(arm_name, targets):
    """Create a planning goal for one arm."""
    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = arm_name
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnect"

    constraints_list = []
    arm_prefix = f"{arm_name}_"

    for i in range(1, 7):
        joint_name = f"{arm_prefix}joint_{i}"
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.weight = 1.0

        if joint_name in targets:
            angle_deg = targets[joint_name]
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


def main():
    rclpy.init()
    node = rclpy.create_node("moveit_test_dual_arm_parallel_plan_execute")
    move_group_client = ActionClient(node, MoveGroup, "/move_action")

    if not move_group_client.wait_for_server(timeout_sec=10):
        print("✗ move_group action server not available!")
        rclpy.shutdown()
        return False

    print("\n" + "=" * 70)
    print("DUAL-ARM 3-PHASE - PARALLEL PLANNING + PARALLEL EXECUTION")
    print("=" * 70)
    print("\nPhase 1: Send ALL 6 planning requests simultaneously")
    print("Phase 2: Wait for ALL 6 planning results")
    print("Phase 3: Queue ALL 6 trajectories simultaneously on controllers")
    print()

    # Define all phases
    phases = [
        (
            1,
            "Home (0°)",
            {
                "arm_1": {
                    "arm_1_joint_1": 0,
                    "arm_1_joint_2": 0,
                    "arm_1_joint_3": 0,
                    "arm_1_joint_4": 0,
                    "arm_1_joint_5": 0,
                    "arm_1_joint_6": 0,
                },
                "arm_2": {
                    "arm_2_joint_1": 0,
                    "arm_2_joint_2": 0,
                    "arm_2_joint_3": 0,
                    "arm_2_joint_4": 0,
                    "arm_2_joint_5": 0,
                    "arm_2_joint_6": 0,
                },
            },
        ),
        (
            2,
            "Joint1=50°, Joint2=-35°",
            {
                "arm_1": {"arm_1_joint_1": 50, "arm_1_joint_2": -35},
                "arm_2": {"arm_2_joint_1": 50, "arm_2_joint_2": -35},
            },
        ),
        (
            3,
            "Joint1=-50°, Joint2=-35°",
            {
                "arm_1": {"arm_1_joint_1": -50, "arm_1_joint_2": -35},
                "arm_2": {"arm_2_joint_1": -50, "arm_2_joint_2": -35},
            },
        ),
    ]

    # ========== PHASE 1: SEND ALL 6 PLANNING REQUESTS SIMULTANEOUSLY ==========
    print("[PHASE 1: SENDING ALL 6 PLANNING REQUESTS SIMULTANEOUSLY]")
    planning_futures = {}

    for phase_num, desc, targets in phases:
        print(f"  Phase {phase_num}: {desc}")
        planning_futures[phase_num] = {}

        # Create goals for both arms
        goal_arm1 = create_planning_goal("arm_1", targets["arm_1"])
        goal_arm2 = create_planning_goal("arm_2", targets["arm_2"])

        # Send BOTH requests simultaneously (non-blocking)
        print(f"    → Sending arm_1 + arm_2 requests...", flush=True)
        planning_futures[phase_num]["arm_1"] = move_group_client.send_goal_async(
            goal_arm1
        )
        planning_futures[phase_num]["arm_2"] = move_group_client.send_goal_async(
            goal_arm2
        )

    # ========== PHASE 2: WAIT FOR ALL 6 PLANNING RESULTS ==========
    print("\n[PHASE 2: WAITING FOR ALL 6 PLANNING RESULTS]")
    trajectories = {}

    for phase_num in [1, 2, 3]:
        print(f"  Phase {phase_num}:")
        trajectories[phase_num] = {}

        # Get arm_1 result
        print(f"    → arm_1...", end=" ", flush=True)
        rclpy.spin_until_future_complete(
            node, planning_futures[phase_num]["arm_1"], timeout_sec=5
        )
        goal_handle_arm1 = planning_futures[phase_num]["arm_1"].result()

        if goal_handle_arm1 is None or not goal_handle_arm1.accepted:
            print("✗ FAILED")
            rclpy.shutdown()
            return False

        result_future_arm1 = goal_handle_arm1.get_result_async()
        rclpy.spin_until_future_complete(node, result_future_arm1, timeout_sec=30)
        result_arm1 = result_future_arm1.result()

        if (
            result_arm1 is None
            or result_arm1.result is None
            or result_arm1.result.error_code.val != 1
        ):
            print("✗ FAILED")
            rclpy.shutdown()
            return False

        trajectories[phase_num]["arm_1"] = result_arm1.result.planned_trajectory
        print("✓")

        # Get arm_2 result
        print(f"    → arm_2...", end=" ", flush=True)
        rclpy.spin_until_future_complete(
            node, planning_futures[phase_num]["arm_2"], timeout_sec=5
        )
        goal_handle_arm2 = planning_futures[phase_num]["arm_2"].result()

        if goal_handle_arm2 is None or not goal_handle_arm2.accepted:
            print("✗ FAILED")
            rclpy.shutdown()
            return False

        result_future_arm2 = goal_handle_arm2.get_result_async()
        rclpy.spin_until_future_complete(node, result_future_arm2, timeout_sec=30)
        result_arm2 = result_future_arm2.result()

        if (
            result_arm2 is None
            or result_arm2.result is None
            or result_arm2.result.error_code.val != 1
        ):
            print("✗ FAILED")
            rclpy.shutdown()
            return False

        trajectories[phase_num]["arm_2"] = result_arm2.result.planned_trajectory
        print("✓")

    # ========== PHASE 3: QUEUE ALL 6 TRAJECTORIES SIMULTANEOUSLY ==========
    print("\n[PHASE 3: QUEUING ALL 6 TRAJECTORIES SIMULTANEOUSLY]")

    # Create controller clients (reuse for all phases)
    arm1_controller = ActionClient(
        node, FollowJointTrajectory, "/arm_1/follow_joint_trajectory"
    )
    if not arm1_controller.wait_for_server(timeout_sec=2.0):
        print("✗ arm_1 controller not available")
        rclpy.shutdown()
        return False

    arm2_controller = ActionClient(
        node, FollowJointTrajectory, "/arm_2/follow_joint_trajectory"
    )
    if not arm2_controller.wait_for_server(timeout_sec=2.0):
        print("✗ arm_2 controller not available")
        rclpy.shutdown()
        return False

    # Queue ALL phases simultaneously (not waiting between them)
    for phase_num in [1, 2, 3]:
        print(f"  Phase {phase_num}:")

        # Queue arm_1 trajectory (non-blocking)
        goal_arm1 = FollowJointTrajectory.Goal()
        goal_arm1.trajectory = trajectories[phase_num]["arm_1"].joint_trajectory
        arm1_controller.send_goal_async(goal_arm1)
        print(f"    → arm_1 queued")

        # Queue arm_2 trajectory (non-blocking)
        goal_arm2 = FollowJointTrajectory.Goal()
        goal_arm2.trajectory = trajectories[phase_num]["arm_2"].joint_trajectory
        arm2_controller.send_goal_async(goal_arm2)
        print(f"    → arm_2 queued")

    # ========== SUMMARY ==========
    print("\n" + "=" * 70)
    print("✓ ALL 6 TRAJECTORIES NOW EXECUTING IN PARALLEL!")
    print("=" * 70)
    print("\nExecution Timeline:")
    print("  arm_1_controller: phase_1a → phase_2a → phase_3a (executing)")
    print("  arm_2_controller: phase_1b → phase_2b → phase_3b (executing)")
    print("\nKey Points:")
    print("  ✓ Planning: ALL 6 requests sent simultaneously")
    print("  ✓ Execution: ALL 6 queued simultaneously on separate controllers")
    print("  ✓ All 3 phases executing in PARALLEL with NO synchronization")
    print()

    rclpy.shutdown()
    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
