#ifndef HARDWARE_DRIVER_HPP
#define HARDWARE_DRIVER_HPP

#include <custom_msgs/HBridgeControls.h>
#include <custom_msgs/MotorControls.h>
#include <custom_msgs/SwitchControls.h>

#include <array>
#include <cstdint>

namespace ros_data
{
extern std::uint8_t                 switch_control;
extern std::array<std::uint16_t, 6> motor_throttles;
extern std::array<std::uint8_t, 6>  servo_angles;
} // namespace ros_data

#endif /* end of include guard: HARDWARE_DRIVER_HPP */
