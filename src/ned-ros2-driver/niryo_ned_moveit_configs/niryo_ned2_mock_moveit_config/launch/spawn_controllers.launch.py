from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    """
    Launch file for spawning ros2_control controllers.

    This launches:
    - Controller spawner to load controllers defined in the robot hardware

    Requires:
    - ros2_control manager running
    - URDF with ros2_control configuration

    Usage: ros2 launch niryo_ned2_mock_moveit_config spawn_controllers.launch.py
    """
    moveit_config = MoveItConfigsBuilder(
        "niryo_ned2_mock", package_name="niryo_ned2_mock_moveit_config"
    ).to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
