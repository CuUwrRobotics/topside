// System includes
#include <stdint.h>
#include <stdio.h>
#include <math.h>

// ROS includes
#include <custom_msgs/ArmJointMotion.h>
#include <custom_msgs/EulerMotion.h>
#include <custom_msgs/Lockout.h>
#include <custom_msgs/ResetMotors.h>
#include <custom_msgs/MotorControls.h>
#include <custom_msgs/HBridgeControls.h>
#include <custom_msgs/SwitchControls.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
// Local includes
#include "joy-mapper-main.hpp"
#include "lockout.hpp"

/* STANDARD MAPPING ***********************************************************
          FORWARDS                     PITCH DOWN
              ^                              ^
              |                              |
              |                              |
TURN LEFT<----+---->TURN RIGHT    ROLL  <----+----> ROLL
              |                   LEFT       |     RIGHT
              |                              |
              v                              v
          BACKWARDS                      PITCH UP
 */

/* DIRECT MOTOR DRIVE MAPPING *************************************************
STICKS:
             MOTOR 0+                          MOTOR 2+
                ^                                 ^
                |                                 |
                |                                 |
  MOTOR 1- <----+----> MOTOR 1+    MOTOR 3-  <----+----> MOTOR 3+
                |                                 |
                |                                 |
                v                                 v
             MOTOR 0-                          MOTOR 2-

PAD:
          MOTOR 4+
             ^
  MOTOR 5- < + > MOTOR 5+
             v
          MOTOR 4-
 */

custom_msgs::EulerMotion robot_motion_vector;
custom_msgs::MotorControls arm_position_vector;
std::array<double, arm_position_vector.motor_throttles.size()>  arm_position_vector_dbl;
ros::Publisher robot_motion_pub;
ros::Publisher arm_motion_pub;
ros::Publisher arm_servo_pub;
ros::Publisher switch_pub;

custom_msgs::MotorControls direct_motor_control;
ros::Publisher motor_control_publisher;

ros::ServiceClient motor_reset_client;
custom_msgs::ResetMotors motor_reset_srv;

ros::ServiceClient lockout_pub;
custom_msgs::Lockout lockout_srv_data;

bool new_joystick_data_ready = false;

// 2D actuators
Joystick_t stick_left, stick_right, pad_left;

// 1D actuators
JoyTrigger_t trig_left, trig_right;

// Buttons
JoyButton_t btn_stick_left, btn_stick_right;
JoyButton_t btn_x, btn_y, btn_a, btn_b;
JoyButton_t btn_back, btn_start, btn_home;
JoyButton_t bumper_left, bumper_right;

// Button history for exiting lockout
LockoutStateMachine lockout_handler(&bumper_left, &bumper_right);

bool USE_DIRECT_MOTOR_DRIVE = false;

void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  stick_left.updown = joy->axes[1];
  stick_left.leftright = joy->axes[0];
  stick_right.updown = joy->axes[4];
  stick_right.leftright = joy->axes[3];
  pad_left.updown = joy->axes[7];
  pad_left.leftright = joy->axes[6];

  trig_left = joy->axes[2];
  trig_right = joy->axes[5];

  btn_a = joy->buttons[0];
  btn_b = joy->buttons[1];
  btn_x = joy->buttons[2];
  btn_y = joy->buttons[3];

  bumper_left = joy->buttons[4];
  bumper_right = joy->buttons[5];

  btn_back = joy->buttons[6];
  btn_start = joy->buttons[7];
  btn_home = joy->buttons[8];

  btn_stick_left = joy->buttons[9];
  btn_stick_right = joy->buttons[10];

  new_joystick_data_ready = true; // Raise flag that data is ready
}

void initialize()
{
  stick_left.updown = 0;
  stick_left.leftright = 0;
  stick_right.updown = 0;
  stick_right.leftright = 0;
  pad_left.updown = 0;
  pad_left.leftright = 0;

  trig_left = 0;
  trig_right = 0;

  btn_a = false;
  btn_b = false;
  btn_x = false;
  btn_y = false;

  bumper_left = false;
  bumper_right = false;

  btn_back = false;
  btn_start = false;
  btn_home = false;

  btn_stick_left = false;
  btn_stick_right = false;

  arm_position_vector_dbl[0] = 0;
  arm_position_vector_dbl[1] = 0;
  arm_position_vector_dbl[2] = 0;

  if (USE_DIRECT_MOTOR_DRIVE)
  {
    for (auto &&throttle : direct_motor_control.motor_throttles)
      throttle = 1500;
  }
}

void pushUpdates()
{
  static bool trigger_motor_reset, prev_lockout = lockout_handler.isLocked();

  trigger_motor_reset = btn_a;

  robot_motion_vector.z = -pad_left.updown;
  robot_motion_vector.x = -stick_left.updown;
  // robot_motion_vector.y = -stick_right.leftright;
  robot_motion_vector.roll = stick_right.leftright;
  robot_motion_vector.pitch = -stick_right.updown;
  robot_motion_vector.yaw = -stick_left.leftright;

  // arm_position_vector.arm_joints[0] = pad_left.leftright;

  float tl = (trig_left + 1) / 2;
  float tr = (trig_right + 1) / 2;

  // Claw Linear Servo
  custom_msgs::HBridgeControls arm_motor_cmd_msg;
  arm_motor_cmd_msg.index = 0; // TODO: These should be configurable :(
  arm_motor_cmd_msg.value = (bumper_left - bumper_right);

  // Switches
  custom_msgs::SwitchControls switch_cmd_msg;
  switch_cmd_msg.index = 0;
  switch_cmd_msg.value = btn_b;

  // arm_position_vector_dbl[0] += (tl - tr) * 0.1;                    // Lift Motor
  arm_position_vector_dbl[1] += (bumper_left - bumper_right) * 0.1; // Wrist Motor
  // arm_position_vector_dbl[2] += (btn_x - btn_y) * 0.1;              // Claw Motor

  // auto clip = [] (double x, double min, double max) -> double { return x > max ? (max) : (x < min ? min: x); };

  // arm_position_vector.motor_throttles[0] = clip(arm_position_vector_dbl[0], 0, 180);
  // arm_position_vector.motor_throttles[1] = clip(arm_position_vector_dbl[1], 0, 180);
  // arm_position_vector.motor_throttles[2] = clip(arm_position_vector_dbl[2], 0, 180);

  if (USE_DIRECT_MOTOR_DRIVE)
  {
    // Direct motor driver
    if (trigger_motor_reset)
    {
      for (auto &&throttle : direct_motor_control.motor_throttles)
        throttle = 1500;
    }
    else
    {
      direct_motor_control.motor_throttles.at(0) = 1500 + (500 * stick_left.updown);
      direct_motor_control.motor_throttles.at(1) = 1500 - (500 * stick_left.leftright);
      direct_motor_control.motor_throttles.at(2) = 1500 + (500 * stick_right.updown);
      direct_motor_control.motor_throttles.at(3) = 1500 - (500 * stick_right.leftright);
      direct_motor_control.motor_throttles.at(4) += (pad_left.updown);
      direct_motor_control.motor_throttles.at(5) -= (pad_left.leftright);

      direct_motor_control.motor_throttles.at(4) = std::max((uint16_t)1000, std::min((uint16_t)2000, direct_motor_control.motor_throttles.at(4)));
      direct_motor_control.motor_throttles.at(5) = std::max((uint16_t)1000, std::min((uint16_t)2000, direct_motor_control.motor_throttles.at(5)));
    }
  }

  // Handle lockout
  // lockout_srv_data.request.lockout = lockout_handler.isLocked();
  // if (prev_lockout != lockout_srv_data.request.lockout) {
  //   lockout_pub.call(lockout_srv_data);
  //   motor_reset_client.call(motor_reset_srv);

  //   if (lockout_srv_data.request.lockout) {
  //     ROS_INFO("Entering lockout. Tap bumpers five times in 3 seconds to continue.\n");
  //   } else {
  //     ROS_INFO("Leaving lockout.\n");
  //   }

  //   prev_lockout = lockout_srv_data.request.lockout;
  // }

  // Push data out
  robot_motion_pub.publish(robot_motion_vector);
  arm_motion_pub.publish(arm_position_vector);
  arm_servo_pub.publish(arm_motor_cmd_msg);
  switch_pub.publish(switch_cmd_msg);


  if (trigger_motor_reset)
  {
    motor_reset_client.call(motor_reset_srv);
  }

  if (USE_DIRECT_MOTOR_DRIVE)
  {
    motor_control_publisher.publish(direct_motor_control);
  }
}

int main(int argc, char *argv[])
{
  printf("Joy Mapper beginning\n");

  ros::init(argc, argv, "joy_mapper");

  ros::Subscriber joy_sub;
  ros::NodeHandle node;

  if (std::find(argv, argv + argc, std::string("-m")) != argv + argc)
  {
    USE_DIRECT_MOTOR_DRIVE = true;
    ROS_WARN("USING DIRECT MOTOR CONTROLS! Stop any other nodes driving motors to avoid glitches.");
    printf("Map:\n");
    printf(" Motor # |    Control     |  Directions  \n");
    printf("---------|----------------|------------- \n");
    printf(" Motor 0 | Left joystick  |  up/down     \n");
    printf(" Motor 1 | Left joystick  |  left/right  \n");
    printf(" Motor 2 | Right joystick |  up/down     \n");
    printf(" Motor 3 | Right joystick |  left/right  \n");
    printf(" Motor 4 | Left pad       |  up/down     \n");
    printf(" Motor 5 | Left pad       |  right/left  \n");
    printf("\nUp/Down -> up is positive throttle\n");
    printf("Left/right -> right is positive throttle\n");
  }

  joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 1, &joyCallback);
  robot_motion_pub = node.advertise<custom_msgs::EulerMotion>("robot_motion/controller_input", 1);
  arm_motion_pub = node.advertise<custom_msgs::MotorControls>("hardware/servo_motors", 1);
  arm_servo_pub = node.advertise<custom_msgs::HBridgeControls>("hardware/h_bridge_controls", 1);
  switch_pub = node.advertise<custom_msgs::SwitchControls>("hardware/switch_controls", 1);

  if (USE_DIRECT_MOTOR_DRIVE)
  {
    motor_control_publisher = node.advertise<custom_msgs::MotorControls>("hardware/main_motors", 1);
  }

  motor_reset_client = node.serviceClient<custom_msgs::ResetMotors>("robot_motion/reset_motor_accelerations");
  lockout_pub = node.serviceClient<custom_msgs::Lockout>("lockout");

  new_joystick_data_ready = false;

  initialize();

  // ROS_INFO("Starting in lockout. Hold bumpers for 3 seconds to begin.\n");

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.001).sleep(); // Sleep 1ms
    if (new_joystick_data_ready)
      pushUpdates();
    // ros::Duration(.001f).sleep();
  }
  return 0;
}
