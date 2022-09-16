import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sparke_description"
    ).find("sparke_description")
    default_model_path = os.path.join(pkg_share, "src/sparke.urdf.xacro")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")
    default_world_path = os.path.join(pkg_share, "src/sparke-test-world.world")

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "sam_bot", "-topic", "robot_description"],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            launch.actions.ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    default_world_path
                ],
                output="screen",
            ),
            launch.actions.DeclareLaunchArgument(
                name='world',
                default_value=default_world_path,
                description='Full path to the world model file to load'
            ),
            joint_state_publisher_node,
            robot_state_publisher_node,
            spawn_entity,
            rviz_node,
        ]
    )
