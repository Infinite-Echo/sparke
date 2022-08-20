# from msilib.schema import Condition
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.conditions import UnlessCondition
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration


# def generate_launch_description():
#     ld = LaunchDescription()

#     desktop = LaunchConfiguration("desktop")
#     declare_desktop_cmd = DeclareLaunchArgument(
#         "desktop", default_value="true", description="Whether to run i2c_pwmboard_node")
#     ld.add_action(declare_desktop_cmd)

#     config = os.path.join(
#         get_package_share_directory("spot_micro_motion_cmd"),
#         "config",
#         "sparke_params.yaml",
#     )

#     node = Node(
#         condition = UnlessCondition(desktop),
#         package="ros_i2cpwm_board",
#         executable="ros_i2cpwm_board",
#         name="ros_i2cpwm_board",
#         parameters=[config],
#     )

#     node = Node(
#         package="spot_micro_motion_cmd",
#         executable="spot_micro_motion_cmd_node",
#         name="spot_micro_motion_cmd",
#         parameters=[config],
#     )

#     ld.add_action(node)
#     return ld

# ####################

# <?xml version="1.0" encoding="utf-8"?>
# <!-- Launch the servo move via keyboard command node, and, optionally, the i2c_pwmboard node -->

# <launch>
#     <!-- Optional command line argument to also run i2c_pwmboard, if running on a rpi  -->
#     <arg name="run_i2c_pwmboard" default="false"/>

#     <!-- Defining the node and executable and publishing the output on terminal-->
#     <node name="servo_move_keyboard_node" pkg="servo_move_keyboard" type="servoMoveKeyboard.py" output="screen">
#     </node>

#     <!-- If run_i2c_pwmboard is true, also run the i2c_pwm_board node -->
#     <node if="$(arg run_i2c_pwmboard)" name="i2cpwm_board_node" pkg="i2cpwm_board" type="i2cpwm_board" output="screen">                                           │walk: Start walk mode and keyboard motion control
#     </node>
# </launch>
