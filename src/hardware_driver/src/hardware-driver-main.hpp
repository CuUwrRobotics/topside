#ifndef HARDWARE_DRIVER_HPP
#define HARDWARE_DRIVER_HPP

#include <custom_msgs/MotorControls.h>
#include <custom_msgs/SwitchControls.h>
#include <custom_msgs/HBridgeControls.h>

#include <array>

namespace ros_data
{
  extern uint8_t switch_control;
  extern std::array<uint16_t, 6> motor_throttles;
  extern std::array<uint8_t, 6> servo_angles;
}

#endif /* end of include guard: HARDWARE_DRIVER_HPP */
