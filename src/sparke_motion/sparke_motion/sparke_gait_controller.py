import numpy as np
from libs.sparkeKinematics.kinematics_np.sparke_leg_IK import SparkeLeg
from geometry_msgs.msg import Twist, Pose
from libs.sparkeKinematics.kinematics_np.base_transformations import create_base_transformation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from libs.sparkeKinematics.kinematics_np.sparke_base_IK import SparkeBase

class SparkeGaitController():
    def __init__(self):
        self.sparkeBase = SparkeBase()
        pass

    def init_vars(self):
        self.cmd_vel = Twist()
        self.zero_vel = Twist()
        self.body_pose = Pose()
        self.gait_phase = 0
        self.Tm = create_base_transformation(0,0,0,0,0,0)

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

    def update_gait(self, target_vel):
        #change bezier curve to match target velocity
        #set gait phase to match last know position in trajectory
        pass
            
    def update_body_pose(self, pose):
        if self.zero_vel == self.cmd_vel:
            self.Tm = create_base_transformation(pose.position.x, pose.position.y, pose.position.z, 0, 0, 0)
        else:
            self.Tm = create_base_transformation(0,0,0,0,0,0)

    def get_next_point(self):
        x_ee_array, y_ee_array, z_ee_array = self.get_leg_end_points()
        goal_positions = self.sparkeBase.get_angles_from_trajectory(self.Tm, x_ee_array, y_ee_array, z_ee_array)
        self.point_msg.positions = goal_positions
        self.traj_msg.points = [self.point_msg]

    def get_leg_end_points(self):
        x_ee_array = np.empty(4)
        y_ee_array = np.empty(4)
        z_ee_array = np.empty(4)
        #use current gait phase to get each legs end points
        return x_ee_array, y_ee_array, z_ee_array        