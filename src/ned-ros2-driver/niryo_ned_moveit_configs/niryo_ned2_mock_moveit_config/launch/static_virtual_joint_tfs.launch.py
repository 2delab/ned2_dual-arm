from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    """
    Launch file for static virtual joint TF frames.

    This launches:
    - Static TF publisher for virtual joint transforms (if defined in SRDF)

    Usage: ros2 launch niryo_ned2_mock_moveit_config static_virtual_joint_tfs.launch.py
    """
    moveit_config = MoveItConfigsBuilder(
        "niryo_ned2_mock", package_name="niryo_ned2_mock_moveit_config"
    ).to_moveit_configs()
    return generate_static_virtual_joint_tfs_launch(moveit_config)
