import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from trajectory_msgs.msg import JointTrajectory
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# from sparke_gait_controller import SparkeGaitController
from .sparke_gait_controller import SparkeGaitController

class SparkeMotionCtrlNode(Node):
    def __init__(self):
        super().__init__("sparke_motion_ctrl_node", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.gait_controller = SparkeGaitController()
        self.init_params()
        self.init_publishers()
        self.init_subscribers()
        self.init_msgs()
        self.init_timers()

    def init_params(self):
        self.publish_rate = 0.5

    def init_publishers(self):
        publish_cb_group = MutuallyExclusiveCallbackGroup()
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10, callback_group=publish_cb_group
        )

    def init_subscribers(self):
        subscriber_cb_group = MutuallyExclusiveCallbackGroup()
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10, callback_group=subscriber_cb_group)
        self.base_pose = self.create_subscription(Pose, 'base_pose', self.base_pose_cb, 10, callback_group=subscriber_cb_group)

    def init_msgs(self):
        self.traj_msg = JointTrajectory()
        self.current_vel_msg = Twist()

    def init_timers(self):
        publish_timer_cb_group = MutuallyExclusiveCallbackGroup()
        gait_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.publish_timer = self.create_timer(self.publish_rate, self.publish_trajectory, callback_group=publish_timer_cb_group)
        self.gait_timer = self.create_timer(self.publish_rate, self.get_traj_point, callback_group=gait_timer_cb_group)

    def cmd_vel_cb(self, cmd_vel):
        if self.current_vel_msg != cmd_vel:
            self.current_vel_msg = cmd_vel
            self.gait_controller.update_gait(self.current_vel_msg)

    def base_pose_cb(self, pose):
        self.gait_controller.update_body_pose(pose)
        
    def publish_trajectory(self):
        self.trajectory_publisher.publish(self.traj_msg)

    def get_traj_point(self):
        self.traj_msg = self.gait_controller.get_next_point()

def main(args=None):
    rclpy.init(args=args)

    motion_ctrl_node = SparkeMotionCtrlNode()

    executor = MultiThreadedExecutor()
    executor.add_node(motion_ctrl_node)

    while rclpy.ok():
        executor.spin()

    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()