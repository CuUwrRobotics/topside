// System includes
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

// ROS includes
// #include <custom_msgs/MotorControls.h>
#include <custom_msgs/EulerMotion.h>
#include <custom_msgs/Leak.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// Internal includes
#include "hardware-driver-main.hpp"
#include "mpu6050.hpp"
#include "ros-callbacks.hpp"
#include "rov-comms.hpp"
#include "serial/serial.hpp"

/// @brief The current time
#define NOW()           (ros::Time::now())
/// Creates a delay in seconds
#define CREATE_DELAY(a) (ros::Time::now() + ros::Duration(a))

void controlLoop(RovCommsController& rov_comms_controller,
                 ros::Publisher&     leak_pub,
                 ros::Publisher&     motion_pub,
                 ros::Publisher&);

constexpr static float MAX_DELAY_SEC = 0.2;

// custom_msgs::MotorControls ros_data::motor_controls_msg;
namespace ros_data
{
std::uint8_t                           g_SwitchControl;
std::array<std::uint16_t, MOTOR_COUNT> g_MotorThrottles;
std::array<std::uint8_t, MOTOR_COUNT>  g_ServoAngles;
} // namespace ros_data

constexpr std::uint8_t MPU_DATA_BYTES = 14;

constexpr std::uint8_t COPI_BUFFER_LENGTH
    = ((2 * ros_data::g_MotorThrottles.size()) + 1
       + ros_data::g_ServoAngles.size());
constexpr std::uint8_t CIPO_BUFFER_LENGTH
    = (1 + MPU_DATA_BYTES + sizeof(float));

custom_msgs::Leak        leak_msg;
custom_msgs::EulerMotion accel_gyro_readings;

ros::Time next_ping_time;

std::uint32_t prev_copi_millis = 0;

enum class Modes : std::uint8_t
{
    HANDSHAKE,
    COPI,
    CIPO_SUCCESSFUL,
    CIPO,
}

Modes mode
    = Modes::CIPO;

int main(int argc, char* argv[])
{
    ROS_INFO("Hardware Driver beginning\n");

    // ROS Initialization
    ros::init(argc, argv, "hardware_driver");
    ros::NodeHandle node;
    ros::Subscriber motor_sub
        = node.subscribe<custom_msgs::MotorControls>("hardware/main_motors",
                                                     1,
                                                     &motionMotorSubCallback);
    ros::Subscriber servo_sub
        = node.subscribe<custom_msgs::MotorControls>("hardware/servo_motors",
                                                     1,
                                                     &servoSubCallback);
    ros::Subscriber switch_sub = node.subscribe<custom_msgs::SwitchControls>(
        "hardware/switch_controls",
        1,
        &switchesSubCallback);
    ros::Subscriber hbridge_sub = node.subscribe<custom_msgs::HBridgeControls>(
        "hardware/h_bridge_controls",
        1,
        &hBridgeSubCallback);

    ros::Publisher leak_pub
        = node.advertise<custom_msgs::Leak>("hardware/leak_sensors", 1);
    ros::Publisher motion_pub
        = node.advertise<custom_msgs::EulerMotion>("hardware/motion", 1);

    ros::Publisher depth_pub
        = node.advertise<std_msgs::Float32>("hardware/depth", 1);

    std::string deviceName = "/dev/ttyUSB0";
    int         baudRate   = 2'000'000;

    if (argc > 1)
    {
        for (std::uint8_t arg = 0; arg < argc; arg++)
        {
            if (std::strcmp(argv[arg], "-d") == 0
                || std::strcmp(argv[arg], "-D") == 0)
            {
                ROS_INFO("-d specified, enabling debug-level logging.");
                if (!ros::console::set_logger_level(
                        ROSCONSOLE_DEFAULT_NAME,
                        ros::console::levels::Debug))
                {
                    ROS_ERROR("Debug logging could not be enabled.");
                }
            }
            else if (std::strcmp(argv[arg], "-p") == 0)
            {
                if (argc < arg + 2)
                {
                    ROS_WARN("-p specified, but no device name was given. "
                             "Continuing with default device name.");
                }
                else
                {
                    deviceName = argv[arg + 1];
                }
            }
            else if (std::strcmp(argv[arg], "-b") == 0)
            {
                if (argc < arg + 2)
                {
                    ROS_WARN("-b specified, but no baud rate was given. "
                             "Continuing with default rate %d.",
                             baudRate);
                }
                else
                {
                    baudRate = atoi(argv[arg + 1]);
                }
            }
        }
    }

    // Arduino Connection
    ROS_INFO("Attempting to open Arduino using device '%s' with baud rate %d\n",
             deviceName.c_str(),
             baudRate);
    int serialFileDescriptor
        = serial::openSerialPort(deviceName.c_str(), baudRate);
    if (serialFileDescriptor < 0)
    {
        ROS_ERROR(
            "Failed to initialize Arduino serial communication. Exiting.\n");
        std::exit(EXIT_FAILURE);
    }

    RovCommsController rov_comms_controller(serialFileDescriptor,
                                            CIPO_BUFFER_LENGTH,
                                            COPI_BUFFER_LENGTH);

    ROS_INFO("Hardware Driver successfully initialized\n");

    rov_comms_controller.awaitHandshake();

    serial::serialEmpty(serialFileDescriptor);

    next_ping_time = CREATE_DELAY(MAX_DELAY_SEC);

    while (ros::ok())
    {
        ros::spinOnce();

        controlLoop(rov_comms_controller, leak_pub, motion_pub, depth_pub);
    } // while ros::ok
}

int last_read_len = 0;

void softReset(RovCommsController& rov_comms_controller)
{
    rov_comms_controller.reset();
    rov_comms_controller.awaitHandshake();
    mode           = Modes::COPI;
    next_ping_time = CREATE_DELAY(MAX_DELAY_SEC);
    last_read_len  = 0;
}

void controlLoop(RovCommsController& rov_comms_controller,
                 ros::Publisher&     leak_pub,
                 ros::Publisher&     motion_pub,
                 ros::Publisher&     depth_pub)
{
    switch (mode)
    {
    case Modes::HANDSHAKE:
        break;

    case Modes::COPI:
        {
            ROS_DEBUG("Mode: COPI");

            // Send Throttle Information
            for (const auto throttle : ros_data::g_MotorThrottles)
            {
                rov_comms_controller.send(throttle);
            }

            rov_comms_controller.send(ros_data::g_SwitchControl);

            // Send Servo Angles
            for (const auto angle : ros_data::g_ServoAngles)
            {
                rov_comms_controller.send(angle);
            }

            ROS_DEBUG("Entering mode: CIPO");
            mode           = Modes::CIPO;
            next_ping_time = CREATE_DELAY(MAX_DELAY_SEC);

            break;
        }

    case Modes::CIPO:
        {
            if (NOW() >= next_ping_time)
            {
                ROS_DEBUG("Timeout: %f", (NOW() - next_ping_time).toSec());
                ROS_WARN("Timeout when communicating with arduino (in CIPO), "
                         "resetting.");
                softReset(rov_comms_controller);
            }

            int read_len = rov_comms_controller.tryReadingData();
            if (read_len == -1)
            {
                ROS_DEBUG("CIPO buffer ready");

                // Make sure there isn't any data remaining in the serial's
                // hardware RX buffer Needed because the arduino will sometimes
                // double-transmit, messing up the synchronization
                serial::serialEmpty(rov_comms_controller.getFileDescriptor(),
                                    TCIFLUSH);

                // Reached the end of the read buffer
                if (rov_comms_controller.checksumGood())
                {
                    mode = Modes::CIPO_SUCCESSFUL;
                }
                else
                {
                    ROS_WARN(
                        "Checksum failed when communicating with arduino (in "
                        "CIPO), resetting.");
                    softReset(rov_comms_controller);
                }
            }
            else if (read_len != last_read_len)
            {
                last_read_len  = read_len;
                next_ping_time = CREATE_DELAY(MAX_DELAY_SEC);
            }
            break;
        }

    case Modes::CIPO_SUCCESSFUL:
        {
            ROS_DEBUG("MODE: CIPO_SUCCESSFUL");

            rov_comms_controller.resetReadBuffer();
            leak_msg.leaks = rov_comms_controller.popReadBuffer();
            leak_pub.publish(leak_msg);

            std::uint8_t* data = rov_comms_controller.popReadBuffer(14);
            motion_pub.publish(processMpu6050Data(data));

            float depth = rov_comms_controller.popReadBuffer<float>();
            std_msgs::Float32 depth_msg;
            depth_msg.data = depth;
            depth_pub.publish(depth_msg);

            mode = Modes::COPI;
            break;
        }

    default:
        {
            ROS_ERROR("Invalid state reached.");
            std::exit(EXIT_FAILURE);
        }
    }
}; // controlLoop
