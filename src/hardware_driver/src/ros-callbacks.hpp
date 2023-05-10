#include "hardware-driver-main.hpp"

void motionMotorSubCallback(const custom_msgs::MotorControls::ConstPtr& cmd);
void servoSubCallback(const custom_msgs::MotorControls::ConstPtr& cmd);
void switchesSubCallback(const custom_msgs::SwitchControls::ConstPtr& cmd);
void hBridgeSubCallback(const custom_msgs::HBridgeControls::ConstPtr& cmd);
