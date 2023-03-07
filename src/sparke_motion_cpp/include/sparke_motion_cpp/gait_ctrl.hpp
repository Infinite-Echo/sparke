#include <functional>
#include <memory>

#include <rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class gait_ctrl : public rclcpp::Node
{
private:
    /* Subscribers */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

    /* Publishers */
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub;

    /* Variables */
    geometry_msgs::msg::Twist current_vel;
    trajectory_msgs::msg::JointTrajectory trajectories; //may need to be an array

public:
    gait_ctrl(rclcpp::NodeOptions options);
    ~gait_ctrl();
    /* Callbacks */
    void cmd_vel_cb(const geometry_msgs::msg::Twist &cmd_vel);
    void change_trajectories();
    void publish_trajectories();
};

