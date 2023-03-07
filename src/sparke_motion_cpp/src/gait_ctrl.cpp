#include <functional>
#include <memory>

#include <rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "gait_ctrl.hpp"
// using std::placeholders::_1;

gait_ctrl::gait_ctrl(rclcpp::NodeOptions options):
Node("spot_micro_motion_cmd", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
{
    RCLCPP_INFO(rclcpp::get_logger("Gait Controller"), "Constructing Gait Controller Node");

    /* Subscribers */
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(gait_ctrl::cmd_vel_cb, this, std::placeholders::_1));

    /* Publishers */
    trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_group_effort_controller/joint_trajectory", 1);

    // NOTE: May be necessary to initialize variables; Ex: trajectories
}

gait_ctrl::~gait_ctrl()
{
    RCLCPP_INFO(rclcpp::get_logger("Gait Controller"), "Destroying Gait Controller Node");
}

void gait_ctrl::cmd_vel_cb(const geometry_msgs::msg::Twist &cmd_vel)
{
    if (cmd_vel != current_vel) { //not sure if it can be compared this way
        current_vel = cmd_vel; //may need to create util function for this
        //add function call to recalculate the 200 trajectory points
        change_trajectories();
    }
}

void gait_ctrl::change_trajectories()
{
    //get 200 points from bezier based on cmd_vel
    //convert 200 points into joint angles using IK
    publish_trajectories();
}

void gait_ctrl::publish_trajectories()
{
    trajectory_pub->publish(trajectories);
}