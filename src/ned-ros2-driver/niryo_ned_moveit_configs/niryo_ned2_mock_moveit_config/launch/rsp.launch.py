from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    """
    Launch file for Robot State Publisher only.

    This launches:
    - Robot State Publisher (publishes TF transforms from joint states)

    Requires:
    - Joint State Publisher or joint state source publishing to /joint_states

    Usage: ros2 launch niryo_ned2_mock_moveit_config rsp.launch.py
    """
    moveit_config = MoveItConfigsBuilder(
        "niryo_ned2_mock", package_name="niryo_ned2_mock_moveit_config"
    ).to_moveit_configs()
    return generate_rsp_launch(moveit_config)
