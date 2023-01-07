import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from .sparke_deep_learning_submodule.sparke_gait_generator import SparkeAI
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
import numpy as np
import tensorflow as tf

class SparkeAINode(Node):
    def __init__(self):
        super().__init__("sparke_ai_node", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.ai = SparkeAI()

        timer_cb_group = MutuallyExclusiveCallbackGroup()
        publisher_cb_group = ReentrantCallbackGroup()
        subscription_cb_group = ReentrantCallbackGroup()
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10, callback_group=publisher_cb_group
        )

        self.init_msgs()

        self.timer = self.create_timer(0.2, self.timer_cb, callback_group=timer_cb_group)
        self.create_subscription(Odometry, "/odom", self.odom_cb, callback_group=subscription_cb_group, qos_profile=qos_policy)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10, callback_group=subscription_cb_group)

    def timer_cb(self):
        actor_state = self.get_actor_state()
        predicted_joint_angles = self.ai.predict_policy(actor_state)
        joint_angles = self.convert_tensor_to_list(predicted_joint_angles)
        self.get_logger().info(f'joint angles: {joint_angles}')
        self.update_state_angles(joint_angles)
        self.publish_trajectory(joint_angles)
        critic_state = self.get_critic_state()
        predicted_value = self.ai.predict_value(critic_state)
        self.get_logger().info(f'predicted value: {predicted_value.numpy()[0]}')

    def odom_cb(self, msg):
        self.state_arr[0][0] = msg.pose.pose.position.x
        self.state_arr[0][1] = msg.pose.pose.position.y
        self.state_arr[0][2] = msg.pose.pose.position.z
        self.state_arr[0][3] = msg.pose.pose.orientation.x
        self.state_arr[0][4] = msg.pose.pose.orientation.y
        self.state_arr[0][5] = msg.pose.pose.orientation.z
        self.state_arr[0][6] = msg.pose.pose.orientation.w
        self.state_arr[0][7] = msg.twist.twist.linear.x
        self.state_arr[0][8] = msg.twist.twist.linear.y
        self.state_arr[0][9] = msg.twist.twist.linear.z
        self.state_arr[0][10] = msg.twist.twist.angular.x
        self.state_arr[0][11] = msg.twist.twist.angular.y
        self.state_arr[0][12] = msg.twist.twist.angular.z

    def cmd_vel_cb(self, msg):
        velocity = []
        velocity.append(msg.linear.x)
        velocity.append(msg.linear.y)
        velocity.append(msg.linear.z)
        velocity.append(msg.angular.x)
        velocity.append(msg.angular.y)
        velocity.append(msg.angular.z)
        self.target_velocity = velocity

    def convert_tensor_to_list(self, tensor):
        joint_angles_arr = tensor.numpy()
        joint_angles_list = [0]*12
        for x in range(12):
            joint_angles_list[x] = float(joint_angles_arr[0][x])
        return joint_angles_list

    def init_msgs(self):
        self.target_velocity = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]
        self.state_arr = np.zeros((1,25), dtype=np.float32)

        #initialize trajectory msgs
        self.point_msg = JointTrajectoryPoint()
        self.trajectory_msg = JointTrajectory()
        initial_joint_angles = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
        self.trajectory_msg.joint_names = joints
        self.update_state_angles(initial_joint_angles)
        self.publish_trajectory(initial_joint_angles)

    def publish_trajectory(self, joint_angles):
        self.point_msg.positions = joint_angles
        self.trajectory_msg.points = [self.point_msg]
        self.trajectory_publisher.publish(self.trajectory_msg)

    def update_state_angles(self, joint_angles):
        for x in range(12):
            self.state_arr[0][x+13] = joint_angles[x]

    def get_actor_state(self):
        ''' 
        State Order: 
            X_pos - 0
            Y_pos - 1
            Z_pos - 2
            X_ang - 3
            Y_ang - 4
            Z_ang - 5
            W_ang - 6
            X_vel - 7
            Y_vel - 8
            Z_vel - 9
            X_ang_vel - 10
            Y_ang_vel - 11
            Z_ang_vel - 12
            Joint Angles -13:24
        '''
        current_state_arr = self.state_arr
        for x in range(6):
            current_state_arr[0][x+7] = self.target_velocity[x]
        return current_state_arr

    def get_critic_state(self):
        ''' 
        State Order: 
            X_pos - 0
            Y_pos - 1
            Z_pos - 2
            X_ang - 3
            Y_ang - 4
            Z_ang - 5
            W_ang - 6
            X_vel - 7
            Y_vel - 8
            Z_vel - 9
            X_ang_vel - 10
            Y_ang_vel - 11
            Z_ang_vel - 12
            Joint Angles -13:24
        '''
        previous_state_arr = self.state_arr
        return previous_state_arr

def main(args=None):
    rclpy.init(args=args)

    ai_node = SparkeAINode()

    executor = MultiThreadedExecutor()
    executor.add_node(ai_node)

    while rclpy.ok():
        executor.spin()

    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()