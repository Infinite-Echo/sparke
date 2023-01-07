import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from .sparke_deep_learning_submodule.sparke_gait_generator import SparkeAI
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist

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
        self.update_trajectory(predicted_joint_angles)
        self.publish_trajectory()
        critic_state = self.get_critic_state()
        predicted_value = self.ai.predict_value(critic_state)

    def odom_cb(self, msg):
        new_pose = []
        new_pose.append(msg.pose.pose.position.x)
        new_pose.append(msg.pose.pose.position.y)
        new_pose.append(msg.pose.pose.position.z)
        new_pose.append(msg.pose.pose.orientation.x)
        new_pose.append(msg.pose.pose.orientation.y)
        new_pose.append(msg.pose.pose.orientation.z)
        new_pose.append(msg.pose.pose.orientation.w)
        new_pose.append(msg.twist.twist.linear.x)
        new_pose.append(msg.twist.twist.linear.y)
        new_pose.append(msg.twist.twist.linear.z)
        new_pose.append(msg.twist.twist.angular.x)
        new_pose.append(msg.twist.twist.angular.y)
        new_pose.append(msg.twist.twist.angular.z)
        self.current_pose = new_pose

    def cmd_vel_cb(self, msg):
        velocity = []
        velocity.append(msg.linear.x)
        velocity.append(msg.linear.y)
        velocity.append(msg.linear.z)
        velocity.append(msg.angular.x)
        velocity.append(msg.angular.y)
        velocity.append(msg.angular.z)
        self.target_velocity = velocity

    def init_msgs(self):
        initial_joint_angles = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]
        self.update_trajectory(initial_joint_angles)
        self.target_velocity = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
        self.point_msg.positions = initial_joint_angles
        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.joint_names = joints
        self.trajectory_msg.points = [self.point_msg]

        self.trajectory_publisher.publish(self.trajectory_msg)

    def publish_trajectory(self):
        self.point_msg.positions = self.current_joint_angles
        self.trajectory_msg.points = [self.point_msg]
        self.trajectory_publisher.publish(self.trajectory_msg)

    def update_trajectory(self, joint_angles):
        self.current_joint_angles = joint_angles

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
        state = []
        state.append(self.current_pose)
        # for x in range(6):
        #     state[x+7] = self.target_velocity[x]
        state.append(self.current_joint_angles)
        return state

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
        state = []
        state.append(self.current_pose)
        state.append(self.current_joint_angles)
        return state

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