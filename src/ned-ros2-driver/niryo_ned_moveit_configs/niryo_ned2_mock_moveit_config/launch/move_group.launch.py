from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    """
    Launch file for MoveIt move_group node only (without RViz or joint state publisher).

    This launches:
    - MoveIt move_group for planning and execution
    - Robot State Publisher (TF transforms)

    Usage: ros2 launch niryo_ned2_mock_moveit_config move_group.launch.py
    """
    moveit_config = MoveItConfigsBuilder(
        "niryo_ned2_mock", package_name="niryo_ned2_mock_moveit_config"
    ).to_moveit_configs()
    return generate_move_group_launch(moveit_config)
