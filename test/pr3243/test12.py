#!/usr/bin/env python3
"""
MoveIt2 Dual-Arm 3-Phase with Separate Planning - TRULY PARALLEL

- Plan arm_1 and arm_2 IN PARALLEL (both planning requests sent simultaneously)
- Execute all phases IN PARALLEL (both controllers execute simultaneously)
- No waiting between arms, true asynchronous
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


def get_planning_result(move_group_client, node, goal_future, timeout=30):
    """Wait for planning result from a future."""
    try:
        rclpy.spin_until_future_complete(node, goal_future, timeout_sec=5)
        goal_handle = goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)
        result_response = result_future.result()

        if result_response is None or result_response.result is None:
            return None

        if result_response.result.error_code.val != 1:
            return None

        return result_response.result.planned_trajectory
    except:
        return None


def main():
    rclpy.init()
    node = rclpy.create_node("moveit_test_dual_arm_parallel")
    move_group_client = ActionClient(node, MoveGroup, "/move_action")

    if not move_group_client.wait_for_server(timeout_sec=10):
        print("✗ move_group action server not available!")
        rclpy.shutdown()
        return False

    print("\n" + "=" * 70)
    print("DUAL-ARM 3-PHASE PARALLEL PLANNING & EXECUTION")
    print("=" * 70)
    print("Planning: arm_1 and arm_2 SIMULTANEOUSLY (not sequentially)")
    print("Execution: All 3 phases queued SIMULTANEOUSLY on separate controllers")
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

    # ========== PLANNING STAGE - SEND BOTH SIMULTANEOUSLY ==========
    print("[PLANNING - SEND REQUESTS SIMULTANEOUSLY]")
    planning_futures = {}

    for phase_num, desc, targets in phases:
        print(f"  Phase {phase_num}: {desc}")
        planning_futures[phase_num] = {}

        # Create goals for both arms
        goal_arm1 = create_planning_goal("arm_1", targets["arm_1"])
        goal_arm2 = create_planning_goal("arm_2", targets["arm_2"])

        # Send BOTH requests simultaneously (non-blocking)
        print(f"    Sending arm_1 + arm_2 requests...", end=" ", flush=True)
        future_arm1 = move_group_client.send_goal_async(goal_arm1)
        future_arm2 = move_group_client.send_goal_async(goal_arm2)
        planning_futures[phase_num]["arm_1"] = future_arm1
        planning_futures[phase_num]["arm_2"] = future_arm2
        print("✓")

    # ========== GET PLANNING RESULTS ==========
    print("\n[WAITING FOR PLANNING RESULTS]")
    trajectories = {}

    for phase_num in [1, 2, 3]:
        print(f"  Phase {phase_num}:")
        trajectories[phase_num] = {}

        # Get results for both arms
        print(f"    arm_1...", end=" ", flush=True)
        traj_arm1 = get_planning_result(
            move_group_client, node, planning_futures[phase_num]["arm_1"]
        )
        if traj_arm1 is None:
            print("✗ FAILED")
            rclpy.shutdown()
            return False
        trajectories[phase_num]["arm_1"] = traj_arm1
        print("✓")

        print(f"    arm_2...", end=" ", flush=True)
        traj_arm2 = get_planning_result(
            move_group_client, node, planning_futures[phase_num]["arm_2"]
        )
        if traj_arm2 is None:
            print("✗ FAILED")
            rclpy.shutdown()
            return False
        trajectories[phase_num]["arm_2"] = traj_arm2
        print("✓")

    # ========== EXECUTION STAGE - QUEUE ALL SIMULTANEOUSLY ==========
    print("\n[QUEUING EXECUTION - ALL PHASES AT ONCE]")

    for phase_num in [1, 2, 3]:
        print(f"  Phase {phase_num}:")

        # Send arm_1 execution
        action_client_arm1 = ActionClient(
            node, FollowJointTrajectory, "/arm_1/follow_joint_trajectory"
        )
        if action_client_arm1.wait_for_server(timeout_sec=2.0):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectories[phase_num]["arm_1"].joint_trajectory
            action_client_arm1.send_goal_async(goal)
            print(f"    arm_1: ✓ queued")
        else:
            print(f"    arm_1: ✗ controller not available")

        # Send arm_2 execution
        action_client_arm2 = ActionClient(
            node, FollowJointTrajectory, "/arm_2/follow_joint_trajectory"
        )
        if action_client_arm2.wait_for_server(timeout_sec=2.0):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectories[phase_num]["arm_2"].joint_trajectory
            action_client_arm2.send_goal_async(goal)
            print(f"    arm_2: ✓ queued")
        else:
            print(f"    arm_2: ✗ controller not available")

    print("\n" + "=" * 70)
    print("✓ ALL PHASES EXECUTING IN PARALLEL!")
    print("=" * 70)
    print("  Phase 1: arm_1 + arm_2 (EXECUTING NOW)")
    print("  Phase 2: arm_1 + arm_2 (EXECUTING NOW)")
    print("  Phase 3: arm_1 + arm_2 (EXECUTING NOW)")
    print("\nKey: Separate planning (arm_1, arm_2)")
    print("     Both arms execute independently on their controllers")
    print()

    rclpy.shutdown()
    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
