#include "ros-callbacks.hpp"
#include "hardware-driver-main.hpp"

#include <ros/ros.h>

void motionMotorSubCallback(const custom_msgs::MotorControls::ConstPtr& cmd) {
  ros_data::motor_controls_msg = (*cmd);
}
