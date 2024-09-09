#include "ros-callbacks.hpp"
#include "hardware-driver-main.hpp"

#include <algorithm>

#include <ros/ros.h>

void motionMotorSubCallback(const custom_msgs::MotorControls::ConstPtr& cmd)
{
    std::copy(cmd->motor_throttles.begin(),
              cmd->motor_throttles.end(),
              ros_data::motor_throttles.begin());
}

void servoSubCallback(const custom_msgs::MotorControls::ConstPtr& cmd)
{
    std::copy(cmd->motor_throttles.begin(),
              cmd->motor_throttles.end(),
              ros_data::servo_angles.begin());
}

void switchesSubCallback(const custom_msgs::SwitchControls::ConstPtr& cmd)
{
    if (cmd->value)
    {
        ros_data::switch_control |= 1u << cmd->index;
    }
    else
    {
        ros_data::switch_control &= ~(1u << cmd->index);
    }
}

void hBridgeSubCallback(const custom_msgs::HBridgeControls::ConstPtr& cmd)
{
    if (cmd->value == 1)
    {
        ros_data::switch_control |= 1u << (cmd->index + 4);
        ros_data::switch_control &= ~(1u << (cmd->index + 5));
    }
    else if (cmd->value == -1)
    {
        ros_data::switch_control &= ~(1u << (cmd->index + 4));
        ros_data::switch_control |= 1u << (cmd->index + 5);
    }
    else
    {
        ros_data::switch_control &= ~(1u << (cmd->index + 4));
        ros_data::switch_control &= ~(1u << (cmd->index + 5));
    }
}
