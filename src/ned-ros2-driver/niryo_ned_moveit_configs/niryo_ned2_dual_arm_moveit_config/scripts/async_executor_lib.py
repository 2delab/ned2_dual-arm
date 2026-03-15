#!/usr/bin/env python3
"""
Async Executor Library - Embedded executor for dual-arm async trajectory execution.

This is a library version of the executor that can be imported and used directly
in Python code, avoiding separate ROS processes and context issues.
"""

import threading
from typing import Optional, Callable, Dict
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class AsyncDualArmExecutor:
    """
    Embedded async executor for dual-arm trajectories.

    Usage:
        executor = AsyncDualArmExecutor(node)
        executor.execute_async("arm_1", trajectory1, callback)
        executor.execute_async("arm_2", trajectory2, callback)
    """

    def __init__(self, node: Node):
        """
        Initialize executor.

        Args:
            node: ROS2 Node for creating action clients
        """
        self.node = node
        self.action_callback_group = ReentrantCallbackGroup()

        # Action clients - send to MoveIt trajectory executor action servers
        self.arm_1_client = ActionClient(
            node,
            FollowJointTrajectory,
            "/arm_1/follow_joint_trajectory",
            callback_group=self.action_callback_group,
        )

        self.arm_2_client = ActionClient(
            node,
            FollowJointTrajectory,
            "/arm_2/follow_joint_trajectory",
            callback_group=self.action_callback_group,
        )

        self.clients = {
            "arm_1": self.arm_1_client,
            "arm_2": self.arm_2_client,
        }

        # Track active executions
        self.active_executions: Dict[str, Dict] = {}
        self.lock = threading.RLock()

    def _split_trajectory_for_arm(
        self, trajectory: JointTrajectory, arm_id: str
    ) -> JointTrajectory:
        """Split combined trajectory to only include this arm's joints."""
        arm_indices = [
            i
            for i, name in enumerate(trajectory.joint_names)
            if name.startswith(arm_id)
        ]

        if not arm_indices:
            return JointTrajectory()

        split_trajectory = JointTrajectory()
        split_trajectory.joint_names = [trajectory.joint_names[i] for i in arm_indices]

        for point in trajectory.points:
            new_point = JointTrajectoryPoint()
            new_point.positions = [point.positions[i] for i in arm_indices]
            new_point.time_from_start = point.time_from_start
            split_trajectory.points.append(new_point)

        return split_trajectory

    def execute_async(
        self,
        arm_id: str,
        trajectory: JointTrajectory,
        callback: Optional[Callable[[str, bool, str], None]] = None,
        timeout_sec: float = 30.0,
    ):
        """
        Execute trajectory asynchronously with callback.

        Args:
            arm_id: "arm_1" or "arm_2"
            trajectory: JointTrajectory to execute
            callback: Function(arm_id, success, error_msg) called when complete
            timeout_sec: Timeout for execution
        """
        # Split trajectory for this arm
        split_traj = self._split_trajectory_for_arm(trajectory, arm_id)

        if not split_traj.joint_names:
            if callback:
                callback(arm_id, False, "No joints found for arm")
            return

        # Get controller client
        client = self.clients[arm_id]

        # Create goal
        goal = FollowJointTrajectory.Goal(trajectory=split_traj)

        # Send goal asynchronously with callbacks
        def on_accepted(future):
            try:
                goal_handle = future.result()

                if not goal_handle.accepted:
                    if callback:
                        callback(arm_id, False, "Goal rejected by controller")
                    return

                # Goal accepted - wait for result
                result_future = goal_handle.get_result_async()

                def on_result(future):
                    try:
                        result = future.result().result
                        success = (
                            result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
                        )
                        error_msg = (
                            f"error_code={result.error_code}" if not success else ""
                        )

                        if callback:
                            callback(arm_id, success, error_msg)
                    except Exception as e:
                        if callback:
                            callback(arm_id, False, str(e))

                result_future.add_done_callback(on_result)

            except Exception as e:
                if callback:
                    callback(arm_id, False, str(e))

        # Send goal
        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(on_accepted)
