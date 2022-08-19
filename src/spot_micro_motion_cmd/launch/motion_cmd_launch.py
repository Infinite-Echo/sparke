import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="spot_micro_motion_cmd",
                executable="spot_micro_motion_cmd_node",
                name="spot_micro_motion_cmd",
            ),
        ]
    )
