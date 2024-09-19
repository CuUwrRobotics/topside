#pragma once

#include <custom_msgs/HBridgeControls.h>
#include <custom_msgs/MotorControls.h>
#include <custom_msgs/SwitchControls.h>

#include <array>
#include <cstdint>

constexpr std::size_t MOTOR_COUNT = 6;

namespace ros_data
{
extern std::uint8_t                           switchControl;
extern std::array<std::uint16_t, MOTOR_COUNT> motorThrottles;
extern std::array<std::uint8_t, MOTOR_COUNT>  servoAngles;
} // namespace ros_data

