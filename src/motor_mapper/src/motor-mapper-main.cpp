// System includes
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <utility>

// ROS includes
#include <custom_msgs/EulerMotion.h>
#include <custom_msgs/Lockout.h>
#include <custom_msgs/MotorControls.h>
#include <custom_msgs/ResetMotors.h>
#include <ros/ros.h>

// Local includes
#include "motor-mapper-main.hpp"

// Tau is the time (in seconds) to go from 0-100% throttle
constexpr double VERTICAL_ACCELERATION_TAU         = 1;
constexpr double HORIZONTAL_MOTOR_ACCELERATION_TAU = 1.5;
// #define VERTICAL_ACCELERATION_TAU 0.01
// #define HORIZONTAL_MOTOR_ACCELERATION_TAU 0.01

constexpr double      VERTICAL_THROTTLE         = 0.3;
constexpr double      HORIZONTAL_MOTOR_THROTTLE = 0.5;
constexpr std::size_t MOTOR_COUNT               = 6;

std::array<std::pair<float, float>, MOTOR_COUNT> motor_throttle_limits = {
    { -1.0, 1.0 },
    { -1.0, 1.0 },
    { -1.0, 1.0 },
    { -1.0, 1.0 },
    { -1.0, 1.0 },
    { -1.0, 1.0 },
};

constexpr static int MOTOR_THROTTLE_ZERO  = 1500;
constexpr static int MOTOR_THROTTLE_DELTA = 500;

ros::Publisher             motor_pub;
custom_msgs::EulerMotion   robot_motion_vector;
custom_msgs::MotorControls motors_message;

Motor_t motors[MOTOR_COUNT];

bool motionDataReceived = false;
bool shouldUpdateMotor  = false;
bool lockedout          = false;

void resetMotors()
{
    // Reset all motors to zero
    for (std::size_t idx = 0; idx < MOTOR_COUNT; idx++)
    {
        motors_message.motor_throttles[idx] = 0;
        if (motors[idx].accelerator != nullptr)
        {
            // Update what the accelerators think the value is to avoid jumpy
            // values
            motors[idx].accelerator->fillMemory(0);
        }
    }
    shouldUpdateMotor = true;
}

void motionCmdCallback(const custom_msgs::EulerMotion::ConstPtr motionCommand)
{
    // Store motion locally
    motionDataReceived  = true;
    robot_motion_vector = motionCommand;
}

bool resetCmdCallback(const custom_msgs::ResetMotors::Request&  request,
                      const custom_msgs::ResetMotors::Response& response)
{
    resetMotors();
    return true;
}

bool lockoutCallback(const custom_msgs::Lockout::Request&  request,
                     const custom_msgs::Lockout::Response& response)
{
    lockedout = req.lockout;
    printf("%d\n", lockedout);
    return true;
}

void setMotorValues()
{
    if (!motionDataReceived)
    {
        return;
    }

    for (std::size_t idx = 0; idx < MOTOR_COUNT; idx++)
    {
        float throttle;
        // Compute throttles using desired motion and known motor positions
        throttle  = motors[idx].dir.x * robot_motion_vector.x;
        throttle += motors[idx].dir.y * robot_motion_vector.y;
        throttle += motors[idx].dir.z * robot_motion_vector.z;

        throttle += motors[idx].angle.roll * robot_motion_vector.roll;
        throttle += motors[idx].angle.pitch * robot_motion_vector.pitch;
        throttle += motors[idx].angle.yaw * robot_motion_vector.yaw;

        // Motors that need acceleration have it applied using their
        // accelerator.
        if (motors[idx].accelerator != nullptr)
        {
            throttle = motors[idx].accelerator->getNextValue(throttle);
        }

        // Scale throttle to imposed limit
        throttle *= motors[idx].power_limit;

        // Perform min/max limiting
        throttle = std::min(throttle, motor_throttle_limits[idx][1]);
        throttle = std::max(throttle, motor_throttle_limits[idx][0]);

        uint16_t set_throttle
            = (throttle * motors[idx].set_delta) + motors[idx].set_zero;

        // Pack the final throttle into the message data
        motors_message.motor_throttles[idx] = set_throttle;
    }
    shouldUpdateMotor = true;
}

void initMotors()
{
    // // directional motors
    // for (int idx = 0; idx < Kmotors_length - 2; idx++) {
    //   // motors[idx].throttle = 0;
    //   motors[idx].power_limit = 1;
    //   motors[idx].type = MT_ESC;
    // }
    // // vertical motors
    // for (int idx = Kmotors_length - 2; idx < Kmotors_length; idx++) {
    //   // motors[idx].throttle = 0;
    //   motors[idx].power_limit = 1;
    //   motors[idx].type = MT_ESC;
    // }

    // double M0_SIGN = 1;
    // motors[0].dir.x = M0_SIGN * -1;
    // motors[0].dir.y = M0_SIGN * -1;
    // motors[0].dir.z = M0_SIGN * 0;
    // motors[0].angle.roll = M0_SIGN * 0;
    // motors[0].angle.pitch = M0_SIGN * 0;
    // motors[0].angle.y = M0_SIGN * -1;
    // motors[0].set_zero = MOTOR_THROTTLE_ZERO;
    // motors[0].set_delta = MOTOR_THROTTLE_DELTA;
    // motors[0].power_limit = HORIZONTAL_MOTOR_THROTTLE;
    // motors[0].accelerator = new
    // acceleration::DerivativeLimiter(HORIZONTAL_MOTOR_ACCELERATION_TAU);

    // double M1_SIGN = +1;
    // motors[1].dir.x = M1_SIGN * -1;
    // motors[1].dir.y = M1_SIGN * +1;
    // motors[1].dir.z = M1_SIGN * 0;
    // motors[1].angle.roll = M1_SIGN * 0;
    // motors[1].angle.pitch = M1_SIGN * 0;
    // motors[1].angle.y = M1_SIGN * +1;
    // motors[1].set_zero = MOTOR_THROTTLE_ZERO;
    // motors[1].set_delta = MOTOR_THROTTLE_DELTA;
    // motors[1].power_limit = HORIZONTAL_MOTOR_THROTTLE;
    // motors[1].accelerator = new
    // acceleration::DerivativeLimiter(HORIZONTAL_MOTOR_ACCELERATION_TAU);

    // double M2_SIGN = +1;
    // motors[2].dir.x = M2_SIGN * -1;
    // motors[2].dir.y = M2_SIGN * -1;
    // motors[2].dir.z = M2_SIGN * 0;
    // motors[2].angle.roll = M2_SIGN * 0;
    // motors[2].angle.pitch = M2_SIGN * 0;
    // motors[2].angle.y = M2_SIGN * +1;
    // motors[2].set_zero = MOTOR_THROTTLE_ZERO;
    // motors[2].set_delta = MOTOR_THROTTLE_DELTA;
    // motors[2].power_limit = HORIZONTAL_MOTOR_THROTTLE;
    // motors[2].accelerator = new
    // acceleration::DerivativeLimiter(HORIZONTAL_MOTOR_ACCELERATION_TAU);

    // double M3_SIGN = 1;
    // motors[3].dir.x = M3_SIGN * -1;
    // motors[3].dir.y = M3_SIGN * +1;
    // motors[3].dir.z = M3_SIGN * 0;
    // motors[3].angle.roll = M3_SIGN * 0;
    // motors[3].angle.pitch = M3_SIGN * 0;
    // motors[3].angle.y = M3_SIGN * -1;
    // motors[3].set_zero = MOTOR_THROTTLE_ZERO;
    // motors[3].set_delta = MOTOR_THROTTLE_DELTA;
    // motors[3].power_limit = HORIZONTAL_MOTOR_THROTTLE;
    // motors[3].accelerator = new
    // acceleration::DerivativeLimiter(HORIZONTAL_MOTOR_ACCELERATION_TAU);

    // double M4_SIGN = +1;
    // motors[4].dir.y = M4_SIGN * 0;
    // motors[4].dir.x = M4_SIGN * 0;
    // motors[4].dir.z = M4_SIGN * +1;
    // motors[4].angle.roll = M4_SIGN * +1;
    // motors[4].angle.pitch = M4_SIGN * 0;
    // motors[4].angle.y = M4_SIGN * 0;
    // motors[4].set_zero = MOTOR_THROTTLE_ZERO;
    // motors[4].set_delta = MOTOR_THROTTLE_DELTA;
    // motors[4].power_limit = VERTICAL_THROTTLE;
    // motors[4].accelerator = new acceleration::Holder(new
    // acceleration::DerivativeLimiter(VERTICAL_ACCELERATION_TAU));

    // double M5_SIGN = -1;
    // motors[5].dir.x = M5_SIGN * 0;
    // motors[5].dir.y = M5_SIGN * 0;
    // motors[5].dir.z = M5_SIGN * +1;
    // motors[5].angle.roll = M5_SIGN * -1;
    // motors[5].angle.pitch = M5_SIGN * 0;
    // motors[5].angle.y = M5_SIGN * 0;
    // motors[5].set_zero = MOTOR_THROTTLE_ZERO;
    // motors[5].set_delta = MOTOR_THROTTLE_DELTA;
    // motors[5].power_limit = VERTICAL_THROTTLE;
    // motors[5].accelerator = new acceleration::Holder(new
    // acceleration::DerivativeLimiter(VERTICAL_ACCELERATION_TAU));

    double M0_SIGN        = -1;
    motors[0].dir.x       = M0_SIGN * 0;
    motors[0].dir.y       = M0_SIGN * 0;
    motors[0].dir.z       = M0_SIGN * 1;
    motors[0].angle.roll  = M0_SIGN * -1;
    motors[0].angle.pitch = M0_SIGN * +1;
    motors[0].angle.yaw   = M0_SIGN * 0;
    motors[0].set_zero    = MOTOR_THROTTLE_ZERO;
    motors[0].set_delta   = MOTOR_THROTTLE_DELTA;
    motors[0].power_limit = VERTICAL_THROTTLE;
    motors[0].accelerator
        = new acceleration::DerivativeLimiter(VERTICAL_ACCELERATION_TAU);

    double M1_SIGN        = +1;
    motors[1].dir.x       = M1_SIGN * 0;
    motors[1].dir.y       = M1_SIGN * 0;
    motors[1].dir.z       = M1_SIGN * 1;
    motors[1].angle.roll  = M1_SIGN * +1;
    motors[1].angle.pitch = M1_SIGN * +1;
    motors[1].angle.yaw   = M1_SIGN * 0;
    motors[1].set_zero    = MOTOR_THROTTLE_ZERO;
    motors[1].set_delta   = MOTOR_THROTTLE_DELTA;
    motors[1].power_limit = VERTICAL_THROTTLE;
    motors[1].accelerator
        = new acceleration::DerivativeLimiter(VERTICAL_ACCELERATION_TAU);

    double M2_SIGN        = -1;
    motors[2].dir.x       = M2_SIGN * 0;
    motors[2].dir.y       = M2_SIGN * 0;
    motors[2].dir.z       = M2_SIGN * 1;
    motors[2].angle.roll  = M2_SIGN * +1;
    motors[2].angle.pitch = M2_SIGN * -1;
    motors[2].angle.yaw   = M2_SIGN * 0;
    motors[2].set_zero    = MOTOR_THROTTLE_ZERO;
    motors[2].set_delta   = MOTOR_THROTTLE_DELTA;
    motors[2].power_limit = VERTICAL_THROTTLE;
    motors[2].accelerator
        = new acceleration::DerivativeLimiter(VERTICAL_ACCELERATION_TAU);

    double M3_SIGN        = 1;
    motors[3].dir.x       = M3_SIGN * 0;
    motors[3].dir.y       = M3_SIGN * 0;
    motors[3].dir.z       = M3_SIGN * 1;
    motors[3].angle.roll  = M3_SIGN * -1;
    motors[3].angle.pitch = M3_SIGN * -1;
    motors[3].angle.yaw   = M3_SIGN * 0;
    motors[3].set_zero    = MOTOR_THROTTLE_ZERO;
    motors[3].set_delta   = MOTOR_THROTTLE_DELTA;
    motors[3].power_limit = VERTICAL_THROTTLE;
    motors[3].accelerator
        = new acceleration::DerivativeLimiter(VERTICAL_ACCELERATION_TAU);

    double M4_SIGN        = +1;
    motors[4].dir.x       = M4_SIGN * -1;
    motors[4].dir.y       = M4_SIGN * -1;
    motors[4].dir.z       = M4_SIGN * 0;
    motors[4].angle.roll  = M4_SIGN * 0;
    motors[4].angle.pitch = M4_SIGN * 0;
    motors[4].angle.yaw   = M4_SIGN * +1;
    motors[4].set_zero    = MOTOR_THROTTLE_ZERO;
    motors[4].set_delta   = MOTOR_THROTTLE_DELTA;
    motors[4].power_limit = HORIZONTAL_MOTOR_THROTTLE;
    motors[4].accelerator = new acceleration::DerivativeLimiter(
        HORIZONTAL_MOTOR_ACCELERATION_TAU);

    double M5_SIGN        = -1;
    motors[5].dir.x       = M5_SIGN * -1;
    motors[5].dir.y       = M5_SIGN * +1;
    motors[5].dir.z       = M5_SIGN * 0;
    motors[5].angle.roll  = M5_SIGN * 0;
    motors[5].angle.pitch = M5_SIGN * 0;
    motors[5].angle.yaw   = M5_SIGN * -1;
    motors[5].set_zero    = MOTOR_THROTTLE_ZERO;
    motors[5].set_delta   = MOTOR_THROTTLE_DELTA;
    motors[5].power_limit = HORIZONTAL_MOTOR_THROTTLE;
    motors[5].accelerator = new acceleration::DerivativeLimiter(
        HORIZONTAL_MOTOR_ACCELERATION_TAU);
}

int main(int argc, char* argv[])
{
    ROS_INFO("Motor Mapper beginning\n");

    ros::init(argc, argv, "motor_mapper");

    ros::Subscriber    joy_sub;
    ros::Subscriber    lockout_sub;
    ros::NodeHandle    node;
    ros::ServiceServer reset_service;
    ros::ServiceServer lockout_service;

    joy_sub = node.subscribe<custom_msgs::EulerMotion>(
        "robot_motion/control_loop_output",
        1,
        &motionCmdCallback);

    motor_pub
        = node.advertise<custom_msgs::MotorControls>("hardware/main_motors", 1);

    reset_service
        = node.advertiseService("robot_motion/reset_motor_accelerations",
                                &resetCmdCallback);

    lockout_service = node.advertiseService("lockout", &lockoutCallback);

    initMotors();

    ROS_DEBUG("Motors initialized, entering loop.\n");

    while (ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();

        setMotorValues();

        if (lockedout)
        {
            resetMotors();
            motor_pub.publish(motors_message);
        }
        else if (shouldUpdateMotor)
        {
            shouldUpdateMotor = false;
            motor_pub.publish(motors_message);
        }
    }
    return 0;
}
