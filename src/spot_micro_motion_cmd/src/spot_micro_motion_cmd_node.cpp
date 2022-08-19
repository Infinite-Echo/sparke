// Node file to create object and initialising the ROS node
#include "spot_micro_motion_cmd.h"
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    /* initialising the ROS node creating node handle
    for regestring it to the master and then private node handle to
    handle the parameters */
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto motion_cmd_node = std::make_shared<SpotMicroMotionCmd>(options);

    // Only proceed if servo configuration publishing succeeds
    if (motion_cmd_node->publishServoConfiguration()) {

        /* Looking for any interupt else it will continue looping */
        // Main loop runs indefinitely unless there is an interupt call
        rclcpp::spin(motion_cmd_node);
    }
    rclcpp::shutdown();
    return 0;
}
