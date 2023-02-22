#ifndef HARDWARE_DRIVER_HPP
#define HARDWARE_DRIVER_HPP

#define REQUEST_LIST_LENGTH 3
// Request list indicies
#define RL_MOTION_MOTORS 0
#define RL_ARM_MOTORS 1
#define RL_LEAK 2
#define RL_POSITION 3

#include <custom_msgs/MotorControls.h>

namespace ros_data {
extern custom_msgs::MotorControls motor_controls_msg;
// custom_msgs::Telemetry telem;
}

#endif /* end of include guard: HARDWARE_DRIVER_HPP */
