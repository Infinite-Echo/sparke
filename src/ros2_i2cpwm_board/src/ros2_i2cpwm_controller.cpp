#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
}

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "i2c_interfaces/msg/servo.hpp"
#include "i2c_interfaces/msg/servo_array.hpp"
#include "i2c_interfaces/msg/servo_config.hpp"
#include "i2c_interfaces/msg/servo_config_array.hpp"
#include "i2c_interfaces/srv/servos_config.hpp"
#include "i2c_interfaces/msg/position.hpp"
#include "i2c_interfaces/msg/position_array.hpp"
#include "i2c_interfaces/srv/drive_mode.hpp"
#include "i2c_interfaces/srv/stop_servos.hpp"
#include "i2c_interfaces/srv/int_value.hpp"

typedef struct _servo_config {
    int center;
    int range;
    int direction;
    int mode_pos;
} servo_config;

#define _BASE_ADDR 0x40
#ifndef _PI
#define _PI 3.14159265358979323846
#endif

enum pwm_regs {
    // Registers/etc.
    __MODE1 = 0x00,
    __MODE2 = 0x01,
    __SUBADR1 = 0x02, // enable sub address 1 support
    __SUBADR2 = 0x03, // enable sub address 2 support
    __SUBADR3 = 0x04, // enable sub address 2 support
    __PRESCALE = 0xFE,
    __CHANNEL_ON_L = 0x06,
    __CHANNEL_ON_H = 0x07,
    __CHANNEL_OFF_L = 0x08,
    __CHANNEL_OFF_H = 0x09,
    __ALL_CHANNELS_ON_L = 0xFA,
    __ALL_CHANNELS_ON_H = 0xFB,
    __ALL_CHANNELS_OFF_L = 0xFC,
    __ALL_CHANNELS_OFF_H = 0xFD,
    __RESTART = 0x80,
    __SLEEP = 0x10, // enable low power mode
    __ALLCALL = 0x01,
    __INVRT = 0x10, // invert the output control logic
    __OUTDRV = 0x04
};

class PwmController : public rclcpp::Node
{
    public:
        PwmController()
        : Node("pwm_controller_node")
        {
            _servo_cmd_sub = this->create_subscription<i2c_interfaces::msg::ServoArray>(
                "servo_commands", 10, std::bind(&PwmController::)
            );
        }
    
    private:
        void servo_cmd_callback(const i2c_interfaces::msg::ServoArray &msg)
        {
            for (std::vector<i2c_interfaces::msg::Servo>::const_iterator sp = msg.servos.begin(); sp != msg.servos.end(); ++sp) 
            {
                int servo = sp->servo;
                int value = sp->value;

                if ((value < 0) || (value > 4096)) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid PWM value %d :: PWM values must be between 0 and 4096", value);
                    continue;
                }
                _set_pwm_interval(servo, 0, value);
                RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "servo[%d] = %d", servo, value);
            }
        }
    }
};