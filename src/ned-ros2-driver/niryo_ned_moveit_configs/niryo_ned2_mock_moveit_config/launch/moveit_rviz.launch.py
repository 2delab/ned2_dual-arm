from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    """
    Launch file for RViz with MoveIt visualization only.

    This launches:
    - RViz with MoveIt plugin for motion planning visualization

    Requires:
    - Robot State Publisher running separately (publishes TF)
    - MoveIt move_group running separately

    Usage: ros2 launch niryo_ned2_mock_moveit_config moveit_rviz.launch.py
    """
    moveit_config = MoveItConfigsBuilder(
        "niryo_ned2_mock", package_name="niryo_ned2_mock_moveit_config"
    ).to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
