"""

Author: David Valencia
Date: 26 / 08 /2021

Describer:  This script publishes the position "angles" of each joint under 
			the topic /joint_trajectory_controller/joint_trajectory
			
            I need to run first the my_doosan_controller.launch  in order to load and start the controllers
            update: also can lauch my launch enviroment file

            in simple terms, move the robot joints to the desired position using a topic and the controller

            Executable name in the setup file: trajectory_points_topic     			
"""

import rclpy

from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from i2c_interfaces.msg import ServoArray
import time


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("topic_desired_trajectory_publisher_node")
        timer_period = 1.0
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10
        )
        self.initialize_msg()
        self.servo_absolute_subscription = self.create_subscription(
            ServoArray, "/servos_absolute", self.servos_absolute_callback, 10
        )
        self.servo_proportional_subscription = self.create_subscription(
            ServoArray, "/servos_proportional", self.servos_absolute_callback, 10
        )
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def initialize_msg(self):
        initial_positions = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        self.point_msg = JointTrajectoryPoint()
        self.point_msg.positions = initial_positions
        self.point_msg.time_from_start = Duration(sec=1)

        # adding newly created point into trajectory message
        joints = [
            "fr_ankle_joint",
            "fr_shoulder_joint",
            "fr_hip_joint",
            "br_ankle_joint",
            "br_shoulder_joint",
            "br_hip_joint",
            "fl_ankle_joint",
            "fl_shoulder_joint",
            "fl_hip_joint",
            "bl_ankle_joint",
            "bl_shoulder_joint",
            "bl_hip_joint",
        ]

        self.my_trajectory_msg = JointTrajectory()
        self.my_trajectory_msg.joint_names = joints
        self.my_trajectory_msg.points.append(self.point_msg)
        # self.my_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        self.trajectory_publisher.publish(self.my_trajectory_msg)

    def servos_absolute_callback(self, msg):
        goal_positions = []
        for x in range(12):
            goal_positions.append(msg.servos[x].value)
            print(msg.servos[x].value)
            # self.get_logger().info(msg.servos[x].value)
        self.point_msg.positions = goal_positions
        self.my_trajectory_msg.points.append(self.point_msg)
        # self.my_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        # self.point_msg.time_from_start = Duration(nanosec=self.count)
        # self.trajectory_publisher.publish(self.my_trajectory_msg)

    def servos_proportional_callback(self, msg):
        goal_positions = []
        for x in range(12):
            goal_positions.append(msg.servos[x].value)
            # self.get_logger().info(msg.servos[x].value)
        self.point_msg.positions = goal_positions
        self.my_trajectory_msg.points.append(self.point_msg)
        # self.my_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        # self.point_msg.time_from_start = Duration(nanosec=self.count)
        # self.trajectory_publisher.publish(self.my_trajectory_msg)

    def timer_callback(self):
        # self.my_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        self.trajectory_publisher.publish(self.my_trajectory_msg)


def main(args=None):

    rclpy.init(args=args)
    joint_trajectory_object = TrajectoryPublisher()

    rclpy.spin(joint_trajectory_object)

    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
