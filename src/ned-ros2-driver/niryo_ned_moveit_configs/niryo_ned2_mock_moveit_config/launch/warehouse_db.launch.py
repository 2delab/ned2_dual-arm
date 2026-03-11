from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    """
    Launch file for MoveIt trajectory database (warehouse_db).

    This launches:
    - MongoDB and warehouse database for storing/retrieving trajectories

    Usage: ros2 launch niryo_ned2_mock_moveit_config warehouse_db.launch.py
    """
    moveit_config = MoveItConfigsBuilder(
        "niryo_ned2_mock", package_name="niryo_ned2_mock_moveit_config"
    ).to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
