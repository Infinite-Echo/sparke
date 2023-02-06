from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from libs.sparkeKinematics.kinematics_np.base_transformations import create_base_transformation
import numpy as np

class sparke_motion_ctrl_node(Node):
    def __init__(self):
        super().__init__("sparke_motion_ctrl_node", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.init_params()
        self.init_publishers()
        self.init_subscribers()
        self.init_msgs()
        self.timer = self.create_timer(self.publish_rate, self.publish_trajectory)

    def init_params(self):
        self.publish_rate = self.get_parameter('publish_rate')

    def init_publishers(self):
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10
        )

    def init_subscribers(self):
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.base_pose = self.create_subscription(Pose, 'base_pose', self.update_base_transform, 10)

    def init_msgs(self):
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

        self.point_msg = JointTrajectoryPoint()
        self.point_msg.positions = initial_positions
        self.traj_msg = JointTrajectory()
        self.traj_msg.joint_names = joints
        self.traj_msg.points = [self.point_msg]
        self.target_vel_msg = Twist()

    def cmd_vel_cb(self, cmd_vel):
        self.target_vel_msg = cmd_vel

    def update_base_transform(self, pose):
        #ignoring angles for now because of quaternions
        self.Tm = create_base_transformation(pose.position.x, pose.position.y, pose.position.z, 0, 0, 0)
        
    def publish_trajectory(self):
        self.trajectory_publisher.publish(self.traj_msg)

