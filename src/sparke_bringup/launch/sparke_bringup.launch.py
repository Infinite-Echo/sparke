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

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    declare_use_simulation = DeclareLaunchArgument(
        "use_simulation",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )
    use_simulation = LaunchConfiguration("use_simulation")

    urdf_model_path = os.path.join(get_package_share_directory("sparke_description"))
    xacro_file = os.path.join(urdf_model_path, "src", "sparke_spot.urdf.xacro")

    i2cpwm_board_node = Node(
        condition=IfCondition(PythonExpression([" not ", use_simulation])),
        package="ros_i2cpwm_board",
        executable="ros_i2cpwm_board",
    )

    # spot_micro_motion_cmd_node = Node(
    #     package="spot_micro_motion_cmd",
    #     executable="spot_micro_motion_cmd_node",
    # )

    # motion_command_ld = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("spot_micro_motion_cmd"),
    #             "launch",
    #             "motion_cmd.launch.py",
    #         )
    #     ),
    # )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", xacro_file])},
            {"use_tf_static": False},
            {"publish_frequency": 200.0},
            {"ignore_timestamp": True},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [
            declare_use_simulation,
            robot_state_publisher_node,
            # motion_command_ld,
            # i2cpwm_board_node,
        ]
    )
