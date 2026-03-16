#!/usr/bin/env python3

import rclpy
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


def create_collision_object(obj_data):
    """Convert YAML data to CollisionObject message."""
    collision_obj = CollisionObject()
    collision_obj.header.frame_id = obj_data.get("header", {}).get("frame_id", "world")
    collision_obj.id = obj_data["id"]
    collision_obj.operation = bytes([obj_data.get("operation", 0)])

    for prim_data in obj_data.get("primitives", []):
        primitive = SolidPrimitive()
        primitive.type = prim_data["type"]
        primitive.dimensions = prim_data["dimensions"]
        collision_obj.primitives.append(primitive)

    for pose_data in obj_data.get("primitive_poses", []):
        pose = Pose()
        pos = pose_data.get("position", {})
        pose.position.x = float(pos.get("x", 0.0))
        pose.position.y = float(pos.get("y", 0.0))
        pose.position.z = float(pos.get("z", 0.0))
        ori = pose_data.get("orientation", {})
        pose.orientation.x = float(ori.get("x", 0.0))
        pose.orientation.y = float(ori.get("y", 0.0))
        pose.orientation.z = float(ori.get("z", 0.0))
        pose.orientation.w = float(ori.get("w", 1.0))
        collision_obj.primitive_poses.append(pose)

    return collision_obj


def load_and_add_collision_objects(moveit_py):
    """Load collision objects from YAML and add to planning scene."""
    scene_file = os.path.join(
        get_package_share_directory("niryo_ned2_dual_arm_moveit_config"),
        "config",
        "workspace_scene.yaml",
    )

    try:
        with open(scene_file, "r") as f:
            scene_data = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"ERROR: Scene file not found: {scene_file}")
        rclpy.shutdown()
        exit(1)
    except yaml.YAMLError as e:
        print(f"ERROR: Failed to parse YAML: {e}")
        rclpy.shutdown()
        exit(1)

    if not scene_data or "collision_objects" not in scene_data:
        print("ERROR: No collision_objects found in scene file")
        rclpy.shutdown()
        exit(1)

    planning_scene_monitor = moveit_py.get_planning_scene_monitor()

    with planning_scene_monitor.read_write() as scene:
        for obj_data in scene_data["collision_objects"]:
            collision_obj = create_collision_object(obj_data)
            scene.apply_collision_object(collision_obj)


rclpy.init()

config_file = os.path.join(
    get_package_share_directory("niryo_ned2_dual_arm_moveit_config"),
    "config",
    "moveit_py_params.yaml",
)

moveit_py = MoveItPy(node_name="moveit_py_test", launch_params_filepaths=[config_file])
load_and_add_collision_objects(moveit_py)

dual_arm = moveit_py.get_planning_component("dual")
robot_model = moveit_py.get_robot_model()

dual_arm.set_start_state_to_current_state()
robot_state = RobotState(robot_model)

target_radians = math.radians(50.0)
robot_state.set_joint_group_positions("arm_1", [target_radians, 0, 0, 0, 0, 0])
robot_state.set_joint_group_positions("arm_2", [target_radians, 0, 0, 0, 0, 0])

dual_arm.set_goal_state(robot_state=robot_state)
plan_result = dual_arm.plan()

if plan_result:
    moveit_py.execute(plan_result.trajectory, controllers=[])

rclpy.shutdown()
