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
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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

    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="sparke")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="True")
    declare_headless = DeclareLaunchArgument("headless", default_value="False")
    # declare_gazebo_world = DeclareLaunchArgument(
    #     "world", default_value=os.path.join(gz_pkg_share, "worlds/default.world")
    # )
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    # paused = LaunchConfiguration("paused")
    # lite = LaunchConfiguration("lite")
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("sparke_description"),
            "params",
            "simple_controller_effort_test.yaml",
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("sparke_description"),
                    "src",
                    "sparke_spot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sparke_description"
    ).find("sparke_description")

    gazebo_config = os.path.join(
        launch_ros.substitutions.FindPackageShare(package="sparke_description").find(
            "sparke_description"
        ),
        "params/gazebo.yaml",
    )

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

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_effort_controller", "--controller-manager", "/controller_manager"],
    )

    angle_joint_controller = Node(
        package="angle_joint_controller",
        executable="angle_joint_controller",
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            declare_headless,
            start_gazebo_server_cmd,
            start_gazebo_client_cmd,
            start_gazebo_spawner_cmd,
            control_node,
            joint_state_broadcaster_spawner,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
            angle_joint_controller
        ]
    )
