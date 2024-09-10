#ifndef HARDWARE_DRIVER_HPP
#define HARDWARE_DRIVER_HPP

#include <custom_msgs/HBridgeControls.h>
#include <custom_msgs/MotorControls.h>
#include <custom_msgs/SwitchControls.h>

#include <array>
#include <cstdint>

constexpr std::size_t MOTOR_COUNT = 6;

namespace ros_data
{
extern std::uint8_t                           g_SwitchControl;
extern std::array<std::uint16_t, MOTOR_COUNT> g_MotorThrottles;
extern std::array<std::uint8_t, MOTOR_COUNT>  g_ServoAngles;
} // namespace ros_data

#endif /* end of include guard: HARDWARE_DRIVER_HPP */
