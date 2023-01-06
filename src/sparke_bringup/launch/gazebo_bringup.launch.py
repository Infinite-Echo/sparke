# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition

import xacro


def generate_launch_description():
    # gazebo_world = LaunchConfiguration("world")
    gz_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sparke_description"
    ).find("sparke_description")
    bringup_pkg_share = launch_ros.substitutions.FindPackageShare(package="sparke_bringup").find("sparke_bringup")

    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="sparke")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="True")
    declare_headless = DeclareLaunchArgument("headless", default_value="False")
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=os.path.join(
            gz_pkg_share, "params/simple_controller_effort_test.yaml"
        ),
    )
    declare_gazebo_file = DeclareLaunchArgument(
        "gazebo_file",
        default_value=os.path.join(
            bringup_pkg_share, "params/gazebo_params.yaml"
        ),
    )
    declare_sparke_reset_manager_file = DeclareLaunchArgument(
        "sparke_reset_manager_file",
        default_value=os.path.join(
            bringup_pkg_share, "params/sparke_reset_manager_params.yaml"
        ),
    )
    # declare_gazebo_world = DeclareLaunchArgument(
    #     "world", default_value=os.path.join(gz_pkg_share, "worlds/default.world")
    # )
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    # paused = LaunchConfiguration("paused")
    # lite = LaunchConfiguration("lite")
    ros_control_file = LaunchConfiguration("ros_control_file")
    gazebo_file = LaunchConfiguration("gazebo_file")
    sparke_reset_manager_file = LaunchConfiguration("sparke_reset_manager_file")

    urdf_model_path = os.path.join(get_package_share_directory("sparke_description"))

    xacro_file = os.path.join(urdf_model_path, "src", "sparke_spot.urdf.xacro")

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sparke_description"
    ).find("sparke_description")

    launch_dir = os.path.join(pkg_share, "launch")

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        cwd=[launch_dir],
        output="screen",
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([" not ", headless])),
        cmd=["gzclient"],
        cwd=[launch_dir],
        output="screen",
    )

    start_gazebo_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            robot_name,
            "-topic",
            "robot_description",
            "-z",
            "0.179",
        ],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "joint_states_controller",
        ],
        output="screen",
    )

    load_joint_trajectory1_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "joint_group_effort_controller",
        ],
        output="screen",
    )

    start_reset_manager_cmd = Node(
        package="sparke_reset_manager",
        executable="sparke_reset_manager",
        output="screen",
        parameters=[sparke_reset_manager_file],
    )

    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            declare_headless,
            declare_ros_control_file,
            declare_gazebo_file,
            declare_sparke_reset_manager_file,
            start_gazebo_server_cmd,
            start_gazebo_client_cmd,
            start_gazebo_spawner_cmd,
            load_joint_state_controller,
            load_joint_trajectory1_controller,
            start_reset_manager_cmd,
        ]
    )
