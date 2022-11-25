import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("spot_micro_motion_cmd"),
        "config",
        "sim_sparke_params.yaml",
    )

    node = Node(
        package="spot_micro_motion_cmd",
        executable="spot_micro_motion_cmd_node",
        name="spot_micro_motion_cmd",
        parameters=[config],
    )

    ld.add_action(node)
    return ld
