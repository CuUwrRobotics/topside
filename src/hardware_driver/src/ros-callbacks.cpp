#include "ros-callbacks.hpp"
#include "hardware-driver-main.hpp"

#include <ros/ros.h>
#include <algorithm>

void motionMotorSubCallback(const custom_msgs::MotorControls::ConstPtr& cmd) {
  std::copy(cmd->motor_throttles.begin(), cmd->motor_throttles.end(), ros_data::motor_throttles.begin());
}
