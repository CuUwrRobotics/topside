// System includes
#include <cmath>
#include <cstdint>
#include <cstdio>

// ROS includes
#include <custom_msgs/ArmJointMotion.h>
#include <custom_msgs/EulerMotion.h>
#include <custom_msgs/HBridgeControls.h>
#include <custom_msgs/Lockout.h>
#include <custom_msgs/MotorControls.h>
#include <custom_msgs/ResetMotors.h>
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

custom_msgs::EulerMotion   robot_motion_vector;
custom_msgs::MotorControls armPositionMessage;
std::array<double, arm_position_vector.motor_throttles.size()> ArmPosition;

ros::Publisher robot_motion_pub;
ros::Publisher arm_motion_pub;
ros::Publisher arm_servo_pub;
ros::Publisher switch_pub;

custom_msgs::MotorControls directMotorControl;
ros::Publisher             motor_control_publisher;

ros::ServiceClient       motor_reset_client;
custom_msgs::ResetMotors motor_reset_srv;

ros::ServiceClient   lockout_pub;
custom_msgs::Lockout lockout_srv_data;

bool new_joystick_data_ready = false;

// 2D actuators
struct Joysticks
{
    Joystick_t left;
    Joystick_t right;
    Joystick_t pad;
};

// 1D actuators

struct Triggers
{
    JoyTrigger_t left;
    JoyTrigger_t right;
};

// Buttons
struct Buttons
{
    JoyButton_t leftJoystick;
    JoyButton_t rightJoystick;

    JoyButton_t x;
    JoyButton_t y;
    JoyButton_t a;
    JoyButton_t b;

    JoyButton_t back;
    JoyButton_t start;
    JoyButton_t home;

    JoyButton_t leftBumper;
    JoyButton_t rightBumper;
};

// Button history for exiting lockout
LockoutStateMachine lockout_handler(&Buttons::leftBumper,
                                    &Buttons::rightBumper);

bool USE_DIRECT_MOTOR_DRIVE = false;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    Joysticks::left.updown     = joy->axes[1];
    Joysticks::left.leftright  = joy->axes[0];
    Joysticks::right.updown    = joy->axes[4];
    Joysticks::right.leftright = joy->axes[3];
    Joysticks::pad.updown      = joy->axes[7];
    Joysticks::pad.leftright   = joy->axes[6];

    Triggers::left  = joy->axes[2];
    Triggers::right = joy->axes[5];

    Buttons::a = joy->buttons[0];
    Buttons::b = joy->buttons[1];
    Buttons::x = joy->buttons[2];
    Buttons::y = joy->buttons[3];

    Buttons::leftBumper  = joy->buttons[4];
    Buttons::rightBumper = joy->buttons[5];

    Buttons::back  = joy->buttons[6];
    Buttons::start = joy->buttons[7];
    Buttons::home  = joy->buttons[8];

    Buttons::leftJoystick  = joy->buttons[9];
    Buttons::rightJoystick = joy->buttons[10];

    new_joystick_data_ready = true; // Raise flag that data is ready
}

void initialize()
{
    Joysticks::left.updown     = 0;
    Joysticks::left.leftright  = 0;
    Joysticks::right.updown    = 0;
    Joysticks::right.leftright = 0;
    Joysticks::pad.updown      = 0;
    Joysticks::pad.leftright   = 0;

    Triggers::left  = 0;
    Triggers::right = 0;

    Buttons::a = false;
    Buttons::b = false;
    Buttons::x = false;
    Buttons::y = false;

    Buttons::leftBumper  = false;
    Buttons::rightBumper = false;

    Buttons::back  = false;
    Buttons::start = false;
    Buttons::home  = false;

    Buttons::leftJoystick  = false;
    Buttons::rightJoystick = false;

    ArmPosition[0] = 0;
    ArmPosition[1] = 0;
    ArmPosition[2] = 0;

    if (USE_DIRECT_MOTOR_DRIVE)
    {
        for (auto&& throttle : directMotorControl.motor_throttles)
        {
            throttle = 1'500;
        }
    }
}

void pushUpdates()
{
    static bool shouldResetMotor;
    static bool previousLockout = lockout_handler.isLocked();

    shouldResetMotor = Buttons::a;

    robot_motion_vector.z     = -Joysticks::pad.updown;
    robot_motion_vector.x     = -Joysticks::left.updown;
    // robot_motion_vector.y = -Joysticks::right.leftright;
    robot_motion_vector.roll  = Joysticks::right.leftright;
    robot_motion_vector.pitch = -Joysticks::right.updown;
    robot_motion_vector.yaw   = -Joysticks::left.leftright;

    // arm_position_vector.arm_joints[0] = Joysticks::pad.leftright;

    const float tl = (Triggers::left + 1) / 2;
    const float tr = (Triggers::right + 1) / 2;

    // Claw Linear Servo
    custom_msgs::HBridgeControls arm_motor_cmd_msg;
    // TODO: These should be configurable :(
    // Aside from Cary: what should be configurable?
    arm_motor_cmd_msg.index = 0;
    arm_motor_cmd_msg.value = (Buttons::leftBumper - Buttons::rightBumper);

    // Switches
    custom_msgs::SwitchControls switch_cmd_msg;
    switch_cmd_msg.index = 0;
    switch_cmd_msg.value = Buttons::b;

    // ArmPosition[0] += (tl - tr) * 0.1;                    // Lift
    // Motor
    ArmPosition[1]
        += (Buttons::leftBumper - Buttons::rightBumper) * 0.1; // Wrist Motor
    // ArmPosition[2] += (Buttons::x - Buttons::y) * 0.1; // Claw
    // Motor

    // auto clip = [] (double x, double min, double max) -> double { return x >
    // max ? (max) : (x < min ? min: x); };

    // arm_position_vector.motor_throttles[0] = clip(ArmPosition[0],
    // 0, 180); arm_position_vector.motor_throttles[1] =
    // clip(ArmPosition[1], 0, 180);
    // arm_position_vector.motor_throttles[2] = clip(ArmPosition[2],
    // 0, 180);

    if (USE_DIRECT_MOTOR_DRIVE)
    {
        // Direct motor driver
        if (shouldResetMotor)
        {
            for (auto&& throttle : directMotorControl.motor_throttles)
            {
                throttle = 1'500;
            }
        }
        else
        {
            directMotorControl.motor_throttles.at(0)
                = 1'500 + (500 * Joysticks::left.updown);
            directMotorControl.motor_throttles.at(1)
                = 1'500 - (500 * Joysticks::left.leftright);
            directMotorControl.motor_throttles.at(2)
                = 1'500 + (500 * Joysticks::right.updown);
            directMotorControl.motor_throttles.at(3)
                = 1'500 - (500 * Joysticks::right.leftright);
            directMotorControl.motor_throttles.at(4) += (Joysticks::pad.updown);
            directMotorControl.motor_throttles.at(5)
                -= (Joysticks::pad.leftright);

            directMotorControl.motor_throttles.at(4)
                = std::max((uint16_t)1'000,
                           std::min((uint16_t)2'000,
                                    directMotorControl.motor_throttles.at(4)));
            directMotorControl.motor_throttles.at(5)
                = std::max((uint16_t)1'000,
                           std::min((uint16_t)2'000,
                                    directMotorControl.motor_throttles.at(5)));
        }
    }

    // Handle lockout
    // lockout_srv_data.request.lockout = lockout_handler.isLocked();
    // if (previousLockout != lockout_srv_data.request.lockout) {
    //   lockout_pub.call(lockout_srv_data);
    //   motor_reset_client.call(motor_reset_srv);

    //   if (lockout_srv_data.request.lockout) {
    //     ROS_INFO("Entering lockout. Tap bumpers five times in 3 seconds to
    //     continue.\n");
    //   } else {
    //     ROS_INFO("Leaving lockout.\n");
    //   }

    //   previousLockout = lockout_srv_data.request.lockout;
    // }

    // Push data out
    robot_motion_pub.publish(robot_motion_vector);
    arm_motion_pub.publish(arm_position_vector);
    arm_servo_pub.publish(arm_motor_cmd_msg);
    switch_pub.publish(switch_cmd_msg);

    if (shouldResetMotor)
    {
        motor_reset_client.call(motor_reset_srv);
    }

    if (USE_DIRECT_MOTOR_DRIVE)
    {
        motor_control_publisher.publish(directMotorControl);
    }
}

int main(int argc, char* argv[])
{
    printf("Joy Mapper beginning\n");

    ros::init(argc, argv, "joy_mapper");

    ros::Subscriber joy_sub;
    ros::NodeHandle node;

    if (std::find(argv, argv + argc, std::string("-m")) != argv + argc)
    {
        USE_DIRECT_MOTOR_DRIVE = true;
        ROS_WARN("USING DIRECT MOTOR CONTROLS! Stop any other nodes driving "
                 "motors to avoid glitches.");
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

    joy_sub          = node.subscribe<sensor_msgs::Joy>("joy", 1, &joyCallback);
    robot_motion_pub = node.advertise<custom_msgs::EulerMotion>(
        "robot_motion/controller_input",
        1);
    arm_motion_pub
        = node.advertise<custom_msgs::MotorControls>("hardware/servo_motors",
                                                     1);
    arm_servo_pub = node.advertise<custom_msgs::HBridgeControls>(
        "hardware/h_bridge_controls",
        1);
    switch_pub = node.advertise<custom_msgs::SwitchControls>(
        "hardware/switch_controls",
        1);

    if (USE_DIRECT_MOTOR_DRIVE)
    {
        motor_control_publisher
            = node.advertise<custom_msgs::MotorControls>("hardware/main_motors",
                                                         1);
    }

    motor_reset_client = node.serviceClient<custom_msgs::ResetMotors>(
        "robot_motion/reset_motor_accelerations");
    lockout_pub = node.serviceClient<custom_msgs::Lockout>("lockout");

    new_joystick_data_ready = false;

    initialize();

    // ROS_INFO("Starting in lockout. Hold bumpers for 3 seconds to begin.\n");

    while (ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.001).sleep(); // Sleep 1ms
        if (new_joystick_data_ready)
        {
            pushUpdates();
        }
        // ros::Duration(.001f).sleep();
    }
    return 0;
}
