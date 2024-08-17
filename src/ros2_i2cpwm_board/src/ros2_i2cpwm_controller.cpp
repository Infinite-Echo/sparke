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
#include <chrono>
#include <vector>

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

#define _BASE_ADDR 0x40
#ifndef _PI
#define _PI 3.14159265358979323846
#endif

#define DEFAULT_PWM_FREQUENCY 50
#define DEFAULT_IO_DEVICE 1
#define DEFAULT_IO_HANDLE 0
#define DEFAULT_NUM_SERVOS 12
#define DEFAULT_BOARD_NUMBER 0

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
        : Node("pwm_controller_node", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
        {
            _servo_cmd_sub = this->create_subscription<i2c_interfaces::msg::ServoArray>(
                "servo_commands", 10, std::bind(&PwmController::servo_cmd_callback, this, std::placeholders::_1)
            );
            this->declare_parameter("controller_io_handle", DEFAULT_IO_HANDLE);
            this->declare_parameter("i2c_device_number", DEFAULT_IO_DEVICE);
            this->declare_parameter("pwm_frequency", DEFAULT_PWM_FREQUENCY);
            this->declare_parameter("num_servos", DEFAULT_NUM_SERVOS);
            this->declare_parameter("board_number", DEFAULT_BOARD_NUMBER);

            this->_controller_io_handle = this->get_parameter("controller_io_handle").as_int();
            this->_controller_io_device = this->get_parameter("i2c_device_number").as_int();
            this->_pwm_frequency = this->get_parameter("pwm_frequency").as_int();
            this->_num_servos = this->get_parameter("num_servos").as_int();
            this->_board_number = this->get_parameter("board_number").as_int();

            this->init_board(this->_board_number);

            device << "/dev/i2c-" << this->_controller_io_device;
            const char* filename = device.str().c_str();
            if ((_controller_io_handle = open(filename, O_RDWR)) < 0) 
            {
                RCLCPP_FATAL(this->get_logger(), "Failed to open I2C bus %s", filename);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), ("I2C bus opened on %s", filename));
            }
        }

        void set_pwm_frequency(int freq)
        {
            int prescale;
            char oldmode, newmode;
            int res;

            this->set_parameter(rclcpp::Parameter("pwm_frequency", freq));

            float osc_clk = 25000000.0; // 25MHz
            prescale = (int) roundf((osc_clk)/(4096 * freq)) - 1;

            RCLCPP_INFO(this->get_logger(), "Setting PWM frequency to %d Hz", freq);

            nanosleep((const struct timespec[]){{1, 000000L}}, NULL);

            oldmode = i2c_smbus_read_byte_data(_controller_io_handle, __MODE1);
            newmode = (oldmode & 0x7F) | 0x10; // sleep

            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __MODE1, newmode)) // go to sleep
                RCLCPP_ERROR(this->get_logger(), "Unable to set PWM controller to sleep mode");

            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __PRESCALE, (int)prescale))
                RCLCPP_ERROR(this->get_logger(), "Unable to set PWM controller prescale");

            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __MODE1, oldmode))
                RCLCPP_ERROR(this->get_logger(), "Unable to set PWM controller to active mode");

            nanosleep((const struct timespec[]){{0, 600000L}}, NULL); // sleep 600 microsec to ensure min 500 microsec

            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __MODE1, oldmode | 0x80))
                RCLCPP_ERROR(this->get_logger(), "Unable to restore PWM controller to active mode");
        }

        void set_pwm_interval(int servo_num, int start, int end)
        {
            if ((servo_num < 0) || (servo_num > 15))
            {
                RCLCPP_ERROR(this->get_logger(), "Recieved Invalid Servo Number: %d. Servo Number Must be Within 0 and 15.", servo_num);
                return;
            }

            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __CHANNEL_ON_L + 4 * servo_num, start & 0xFF))
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM start low byte on servo %d on board %d", servo_num, this->_board_number);
            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __CHANNEL_ON_H + 4 * servo_num, start >> 8))
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM start high byte on servo %d on board %d", servo_num, this->_board_number);
            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __CHANNEL_OFF_L + 4 * servo_num, end & 0xFF))
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM end low byte on servo %d on board %d", servo_num, this->_board_number);
            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __CHANNEL_OFF_H + 4 * servo_num, end >> 8))
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM end high byte on servo %d on board %d", servo_num, this->_board_number);
        }

        void set_pwm_interval_all(int start, int end)
        {
            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __ALL_CHANNELS_ON_L, start & 0xFF))
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM start low byte for all servos on board %d", this->_board_number);
            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __ALL_CHANNELS_ON_H, start >> 8))
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM start high byte for all servos on board %d", this->_board_number);
            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __ALL_CHANNELS_OFF_L, end & 0xFF))
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM end low byte for all servos on board %d", this->_board_number);
            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __ALL_CHANNELS_OFF_H, end >> 8))
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting PWM end high byte for all servos on board %d", this->_board_number);
        }

        void init_board(int board_num)
        {
            char data;
            if ((board_num < 0) || (board_num > 61))
            {
                RCLCPP_ERROR(this->get_logger(), "Recieved Invalid Board Number: %d. Board Number Must be Within 0 and 61.", board_num);
                return;
            }

            if (0 > ioctl(_controller_io_handle, I2C_SLAVE, (_BASE_ADDR + (board_num)))) {
                RCLCPP_FATAL(this->get_logger(), "Failed to acquire bus access and/or talk to I2C slave at address 0x%02X", (_BASE_ADDR + board_num));
                return; 
            }

            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __MODE2, __OUTDRV))
                RCLCPP_ERROR(this->get_logger(), "Failed to enable PWM outputs for totem-pole structure");

            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __MODE1, __ALLCALL))
                RCLCPP_ERROR(this->get_logger(), "Failed to enable ALLCALL for PWM channels");

            nanosleep((const struct timespec[]){{0, 600000L}}, NULL); // sleep 600 microsec to ensure min 500 microsec

            data = i2c_smbus_read_byte_data(_controller_io_handle, __MODE1);
            data = data & ~__SLEEP; //                 # wake up (reset sleep)

            if (0 > i2c_smbus_write_byte_data(_controller_io_handle, __MODE1, data))
                RCLCPP_ERROR(this->get_logger(), "Failed to recover from low power mode");

            nanosleep((const struct timespec[]){{0, 600000L}}, NULL); // sleep 600 microsec to ensure min 500 microsec

            // the first time we activate a board, we mark it and set all of its servo channels to 0
            this->set_pwm_interval_all(0, 0);
        }
    
    private:
        void servo_cmd_callback(const i2c_interfaces::msg::ServoArray &msg)
        {
            for (std::vector<i2c_interfaces::msg::Servo>::const_iterator sp = msg.servos.begin(); sp != msg.servos.end(); ++sp) 
            {
                int servo = sp->servo;
                int value = sp->value;

                if ((value < 0) || (value > 4096)) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid PWM value %d :: PWM values must be between 0 and 4096", value);
                    continue;
                }
                this->set_pwm_interval(servo, 0, value);
                RCLCPP_DEBUG(this->get_logger(), "servo[%d] = %d", servo, value);
            }
        }

        rclcpp::Subscription<i2c_interfaces::msg::ServoArray>::SharedPtr _servo_cmd_sub;
        int _controller_io_handle; 
        int _controller_io_device;
        int _pwm_frequency; 
        int _num_servos;
        int _board_number;
        std::stringstream device;
};

int main(int argc, char **argv) {
    // initialize the ROS2 node
    rclcpp::init(argc, argv);

    std::shared_ptr<PwmController> node = std::make_shared<PwmController>();

    rclcpp::spin(node);

    close(node->get_parameter("i2c_device_number").as_int());

    rclcpp::shutdown();

    return 0;
}