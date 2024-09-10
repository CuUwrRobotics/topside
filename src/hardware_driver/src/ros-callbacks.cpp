#include <algorithm>

#include "hardware-driver-main.hpp"
#include "ros-callbacks.hpp"

#include <ros/ros.h>

void motionMotorSubCallback(const custom_msgs::MotorControls::ConstPtr& cmd)
{
    std::copy(cmd->motor_throttles.begin(),
              cmd->motor_throttles.end(),
              ros_data::g_MotorThrottles.begin());
}

void servoSubCallback(const custom_msgs::MotorControls::ConstPtr& cmd)
{
    std::copy(cmd->motor_throttles.begin(),
              cmd->motor_throttles.end(),
              ros_data::g_ServoAngles.begin());
}

void switchesSubCallback(const custom_msgs::SwitchControls::ConstPtr& cmd)
{
    if (cmd->value)
    {
        ros_data::g_SwitchControl |= 1u << cmd->index;
    }
    else
    {
        ros_data::g_SwitchControl &= ~(1u << cmd->index);
    }
}

void hBridgeSubCallback(const custom_msgs::HBridgeControls::ConstPtr& cmd)
{
    if (cmd->value == 1)
    {
        ros_data::g_SwitchControl |= 1u << (cmd->index + 4);
        ros_data::g_SwitchControl &= ~(1u << (cmd->index + 5));
    }
    else if (cmd->value == -1)
    {
        ros_data::g_SwitchControl &= ~(1u << (cmd->index + 4));
        ros_data::g_SwitchControl |= 1u << (cmd->index + 5);
    }
    else
    {
        ros_data::g_SwitchControl &= ~(1u << (cmd->index + 4));
        ros_data::g_SwitchControl &= ~(1u << (cmd->index + 5));
    }
}
