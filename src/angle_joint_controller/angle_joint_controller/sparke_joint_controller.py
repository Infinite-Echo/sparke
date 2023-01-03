import rclpy
import sys
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from i2c_interfaces.msg import ServoArray
import time


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("sparke_joint_controller", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        timer_period = 0.1
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10
        )
        self.initialize_msg()
        self.servo_absolute_subscription = self.create_subscription(
            ServoArray, "/servos_absolute", self.servos_absolute_callback, 10
        )
        self.servo_proportional_subscription = self.create_subscription(
            ServoArray, "/servos_proportional", self.servos_proportional_callback, 10
        )
        # self.timer = self.create_timer(timer_period, self.timer_callback)

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
        # self.point_msg.time_from_start = Duration(sec=1)

        # adding newly created point into trajectory message
        joints = [
            "fr_ankle_joint",
            "fr_shoulder_joint",
            "fr_hip_joint",
            "br_ankle_joint",
            "br_shoulder_joint",
            "br_hip_joint",
            "bl_ankle_joint",
            "bl_shoulder_joint",
            "bl_hip_joint",
            "fl_ankle_joint",
            "fl_shoulder_joint",
            "fl_hip_joint",
        ]

        self.my_trajectory_msg = JointTrajectory()
        self.my_trajectory_msg.joint_names = joints
        self.my_trajectory_msg.points = [self.point_msg]
        # self.my_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        self.trajectory_publisher.publish(self.my_trajectory_msg)

    def servos_absolute_callback(self, msg):
        goal_positions = []
        for x in range(12):
            goal_positions.append(msg.servos[x].value)
            print("absolute: ", msg.servos[x].value)
        self.point_msg.positions = goal_positions
        self.my_trajectory_msg.points = [self.point_msg]
        self.trajectory_publisher.publish(self.my_trajectory_msg)
        time.sleep(0.1)

    def servos_proportional_callback(self, msg):
        goal_positions = []
        for x in range(12):
            goal_positions.append(msg.servos[x].value)
            print("proportional: ", msg.servos[x].value)
        self.point_msg.positions = goal_positions
        self.my_trajectory_msg.points = [self.point_msg]
        self.trajectory_publisher.publish(self.my_trajectory_msg)
        time.sleep(0.1)

    def timer_callback(self):
        print("skip")
        # self.trajectory_publisher.publish(self.my_trajectory_msg)


def main(args=None):

    rclpy.init(args=args)
    joint_trajectory_object = TrajectoryPublisher()

    rclpy.spin(joint_trajectory_object)

    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
