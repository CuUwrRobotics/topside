// System includes
#include <stdint.h>
#include <stdio.h>

#include <math.h>

// ROS includes
#include <custom_msgs/EulerMotion.h>
#include <custom_msgs/Lockout.h>
#include <custom_msgs/MotorControls.h>
#include <custom_msgs/ResetMotors.h>
#include <ros/ros.h>

// Local includes
#include "motor-mapper-main.hpp"

// Tau is the time (in seconds) to go from 0-100% throttle
#define VERTICAL_ACCELERATION_TAU 2
#define HORIZONTAL_MOTOR_ACCELERATION_TAU 0.5

#define VERTICAL_THROTTLE 0.75
#define HORIZONTAL_MOTOR_THROTTLE 0.5

const static int MOTOR_THROTTLE_ZERO = 1500;
const static int MOTOR_THROTTLE_DELTA = 500;

custom_msgs::EulerMotion robot_motion_vector;
ros::Publisher motor_pub;
custom_msgs::MotorControls motors_message;

Motor_t motors[6];

bool recieved_motion_data_yet = false;
bool motor_update_flag = false;
bool lockout_flag = false;

const int Kmotors_length = sizeof(motors) / sizeof(motors[0]);

void resetMotors()
{
  // Reset all motors to zero
  for (int i = 0; i < Kmotors_length; i++)
  {
    motors_message.motor_throttles[i] = 0;
    if (motors[i].accelerator != nullptr)
    {
      // Update what the accelerators think the value is to avoid jumpy values
      motors[i].accelerator->fillMemory(0);
    }
  }
  motor_update_flag = true;
}

void motionCmdCallback(const custom_msgs::EulerMotion::ConstPtr &motion_cmd)
{
  // Store motion locally
  recieved_motion_data_yet = true;
  robot_motion_vector = *motion_cmd;
}

bool resetCmdCallback(custom_msgs::ResetMotors::Request &req, custom_msgs::ResetMotors::Response &res)
{
  resetMotors();
  return true;
}

bool lockoutCallback(custom_msgs::Lockout::Request &req, custom_msgs::Lockout::Response &res)
{
  lockout_flag = req.lockout;
  printf("%d\n", lockout_flag);
  return true;
}

void setMotorValues()
{
  if (!recieved_motion_data_yet)
    return;

  float throttle;

  for (int i = 0; i < Kmotors_length; i++)
  {
    // Compute throttles using desired motion and known motor positions
    throttle = motors[i].dir.x * robot_motion_vector.x;
    throttle += motors[i].dir.y * robot_motion_vector.y;
    throttle += motors[i].dir.z * robot_motion_vector.z;

    throttle += motors[i].angle.r * robot_motion_vector.roll;
    throttle += motors[i].angle.p * robot_motion_vector.pitch;
    throttle += motors[i].angle.y * robot_motion_vector.yaw;

    // Motors that need acceleration have it applied using their accelerator.
    if (motors[i].accelerator != nullptr)
    {
      throttle = motors[i].accelerator->getNextValue(throttle);
    }

    // Perform min/max limiting
    if (throttle > 1)
      throttle = 1;
    if (throttle < -1)
      throttle = -1;

    // Scale throttle to imposed limit
    throttle *= motors[i].power_limit;

    // if (motors[i].accelerator != nullptr) {
    //   // Update what the accelerators think the value is to avoid overflows
    //   motors[i].accelerator->fillMemory(throttle);
    // }

    uint16_t set_throttle = (throttle * motors[i].set_delta) + motors[i].set_zero;

    // Only sends message when a change actually occured
    if (motors_message.motor_throttles[i] != set_throttle)
    {
      // Pack the final throttle into the message data
      motors_message.motor_throttles[i] = set_throttle;
      motor_update_flag = true;
    }
  }
}

void initMotors()
{
  // // directional motors
  // for (int i = 0; i < Kmotors_length - 2; i++) {
  //   // motors[i].throttle = 0;
  //   motors[i].power_limit = 1;
  //   motors[i].type = MT_ESC;
  // }
  // // vertical motors
  // for (int i = Kmotors_length - 2; i < Kmotors_length; i++) {
  //   // motors[i].throttle = 0;
  //   motors[i].power_limit = 1;
  //   motors[i].type = MT_ESC;
  // }
  
  using std::cos;
  using std::sin;

  motors[0].dir.x = 0;
  motors[0].dir.y = 0;
  motors[0].dir.z = 1;
  motors[0].angle.r = -1;
  motors[0].angle.p = +1;
  motors[0].angle.y = -1;
  motors[0].set_zero = MOTOR_THROTTLE_ZERO;
  motors[0].set_delta = MOTOR_THROTTLE_DELTA;
  motors[0].power_limit = HORIZONTAL_MOTOR_THROTTLE;
  motors[0].accelerator = new acceleration::DerivativeLimiter(HORIZONTAL_MOTOR_ACCELERATION_TAU);
  
  // M! REVERSED
  motors[1].dir.x = 0;
  motors[1].dir.y = 0;
  motors[1].dir.z = -1; // +1
  motors[1].angle.r = -1; // +1
  motors[1].angle.p = -1; // +1
  motors[1].angle.y = -1; // +1
  motors[1].set_zero = MOTOR_THROTTLE_ZERO;
  motors[1].set_delta = MOTOR_THROTTLE_DELTA;
  motors[1].power_limit = HORIZONTAL_MOTOR_THROTTLE;
  motors[1].accelerator = new acceleration::DerivativeLimiter(HORIZONTAL_MOTOR_ACCELERATION_TAU);

  motors[2].dir.x = 0;
  motors[2].dir.y = 0;
  motors[2].dir.z = 1;
  motors[2].angle.r = +1;
  motors[2].angle.p = -1;
  motors[2].angle.y = -1;
  motors[2].set_zero = MOTOR_THROTTLE_ZERO;
  motors[2].set_delta = MOTOR_THROTTLE_DELTA;
  motors[2].power_limit = HORIZONTAL_MOTOR_THROTTLE;
  motors[2].accelerator = new acceleration::DerivativeLimiter(HORIZONTAL_MOTOR_ACCELERATION_TAU);

  // MOTOR 3 IS BACKWARDS
  motors[3].dir.x = 0;
  motors[3].dir.y = 0;
  motors[3].dir.z = 1;
  motors[3].angle.r = +1; // -1;
  motors[3].angle.p = +1; // -1;
  motors[3].angle.y = -1; // +1;
  motors[3].set_zero = MOTOR_THROTTLE_ZERO;
  motors[3].set_delta = MOTOR_THROTTLE_DELTA;
  motors[3].power_limit = HORIZONTAL_MOTOR_THROTTLE;
  motors[3].accelerator = new acceleration::DerivativeLimiter(HORIZONTAL_MOTOR_ACCELERATION_TAU);

  motors[4].dir.y = 0;
  motors[4].dir.x = 0;
  motors[4].dir.z = 0;
  motors[4].angle.r = 0;
  motors[4].angle.p = 0;
  motors[4].angle.y = 0;
  motors[4].set_zero = MOTOR_THROTTLE_ZERO;
  motors[4].set_delta = MOTOR_THROTTLE_DELTA;
  motors[4].power_limit = VERTICAL_THROTTLE;
  motors[4].accelerator = new acceleration::Holder(new acceleration::DerivativeLimiter(VERTICAL_ACCELERATION_TAU));

  motors[5].dir.x = 0;
  motors[5].dir.y = 0;
  motors[5].dir.z = 0;
  motors[5].angle.r = 0;
  motors[5].angle.p = 0;
  motors[5].angle.y = 0;
  motors[5].set_zero = MOTOR_THROTTLE_ZERO;
  motors[5].set_delta = MOTOR_THROTTLE_DELTA;
  motors[5].power_limit = VERTICAL_THROTTLE;
  motors[5].accelerator = new acceleration::Holder(new acceleration::DerivativeLimiter(VERTICAL_ACCELERATION_TAU));
}

int main(int argc, char *argv[])
{
  ROS_INFO("Motor Mapper beginning\n");

  ros::init(argc, argv, "motor_mapper");

  ros::Subscriber joy_sub;
  ros::Subscriber lockout_sub;
  ros::NodeHandle node;
  ros::ServiceServer reset_service;
  ros::ServiceServer lockout_service;

  joy_sub = node.subscribe<custom_msgs::EulerMotion>("robot_motion/controller_input", 1, &motionCmdCallback);

  motor_pub = node.advertise<custom_msgs::MotorControls>("hardware/main_motors", 1);

  reset_service = node.advertiseService("robot_motion/reset_motor_accelerations", &resetCmdCallback);

  lockout_service = node.advertiseService("lockout", &lockoutCallback);

  initMotors();

  ROS_DEBUG("Motors initialized, entering loop.\n");

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();

    setMotorValues();

    if (lockout_flag)
    {
      resetMotors();
      motor_pub.publish(motors_message);
    }
    else if (motor_update_flag)
    {
      motor_update_flag = false;
      motor_pub.publish(motors_message);
    }
  }
  return 0;
}
