import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")
    base_frame = "base_link"

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sparke_bringup"
    ).find("sparke_bringup")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sparke_description"
    ).find("sparke_description")
    ros_control_config = os.path.join(descr_pkg_share, "params/ros_control.yaml")
    default_model_path = os.path.join(descr_pkg_share, "src/sparke_spot.urdf.xacro")
    # default_world_path = os.path.join(config_pkg_share, "worlds/playground.world")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="sparke", description="Robot name"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    # declare_gazebo_world = DeclareLaunchArgument(
    #     "world", default_value=default_world_path, description="Gazebo world name"
    # )

    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sparke_bringup"),
                "launch",
                "sparke_bringup.launch.py",
            )
        ),
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sparke_bringup"),
                "launch",
                "gazebo_bringup.launch.py",
            )
        ),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_ros_control_file,
            # declare_gazebo_world,
            bringup_ld,
            gazebo_ld,
        ]
    )
