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

void controlLoop(RovCommsController& rov_comms_controller,
                 ros::Publisher&     leak_pub,
                 ros::Publisher&     motion_pub,
                 ros::Publisher&);

constexpr static float MAX_DELAY_SEC = 0.2;

// custom_msgs::MotorControls ros_data::motor_controls_msg;
namespace ros_data
{
std::uint8_t                           switchControl;
std::array<std::uint16_t, MOTOR_COUNT> motorThrottles;
std::array<std::uint8_t, MOTOR_COUNT>  servoAngles;
} // namespace ros_data

constexpr std::uint8_t MPU_DATA_BYTES = 14;

constexpr std::uint8_t COPI_BUFFER_LENGTH
    = ((2 * ros_data::motorThrottles.size()) + 1
       + ros_data::servoAngles.size());
constexpr std::uint8_t CIPO_BUFFER_LENGTH
    = (1 + MPU_DATA_BYTES + sizeof(float));

custom_msgs::Leak        leakMessage;
custom_msgs::EulerMotion accelerometerGyroReadings;

ros::Time nextPingTime;

std::uint32_t previousCopiTime = 0;

enum class Modes : std::uint8_t
{
    HANDSHAKE,
    COPI,
    CIPO_SUCCESSFUL,
    CIPO,
};

Modes mode = Modes::CIPO;

int main(int argc, char* argv[])
{
    ROS_INFO("Hardware Driver beginning\n");

    // ROS Initialization
    ros::init(argc, argv, "hardware_driver");
    ros::NodeHandle node;

    constexpr std::size_t MAX_QUEUE_SIZE = 1;

    ros::Subscriber motorSubscriber
        = node.subscribe<custom_msgs::MotorControls>("hardware/main_motors",
                                                     MAX_QUEUE_SIZE,
                                                     &motionMotorSubCallback);
    ros::Subscriber servoSubscriber
        = node.subscribe<custom_msgs::MotorControls>("hardware/servo_motors",
                                                     MAX_QUEUE_SIZE,
                                                     &servoSubCallback);
    ros::Subscriber switchSubscriber
        = node.subscribe<custom_msgs::SwitchControls>(
            "hardware/switch_controls",
            MAX_QUEUE_SIZE,
            &switchesSubCallback);
    ros::Subscriber hBridgeSubscriber
        = node.subscribe<custom_msgs::HBridgeControls>(
            "hardware/h_bridge_controls",
            MAX_QUEUE_SIZE,
            &hBridgeSubCallback);

    ros::Publisher leak_pub
        = node.advertise<custom_msgs::Leak>("hardware/leak_sensors", 1);
    ros::Publisher motion_pub
        = node.advertise<custom_msgs::EulerMotion>("hardware/motion", 1);
    ros::Publisher depth_pub
        = node.advertise<std_msgs::Float32>("hardware/depth", 1);

    std::string deviceName = "/dev/ttyUSB0";
    int         baudRate   = 2000000;

    if (argc > 1)
    {
        for (std::size_t idx = 0; idx < argc; idx++)
        {
            if (std::strcmp(argv[idx], "-d") == 0
                || std::strcmp(argv[idx], "-D") == 0)
            {
                ROS_INFO("-d specified, enabling debug-level logging.");
                if (!ros::console::set_logger_level(
                        ROSCONSOLE_DEFAULT_NAME,
                        ros::console::levels::Debug))
                {
                    ROS_ERROR("Debug logging could not be enabled.");
                }
            }
            else if (std::strcmp(argv[idx], "-p") == 0)
            {
                if (argc < idx + 2)
                {
                    ROS_WARN("-p specified, but no device name was given. "
                             "Continuing with default device name.");
                }
                else
                {
                    deviceName = argv[idx + 1];
                }
            }
            else if (std::strcmp(argv[idx], "-b") == 0)
            {
                if (argc < idx + 2)
                {
                    ROS_WARN("-b specified, but no baud rate was given. "
                             "Continuing with default rate %d.",
                             baudRate);
                }
                else
                {
                    baudRate = std::atoi(argv[idx + 1]);
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

    RovCommsController rovCommsController(serialFileDescriptor,
                                          CIPO_BUFFER_LENGTH,
                                          COPI_BUFFER_LENGTH);

    ROS_INFO("Hardware Driver successfully initialized\n");

    rovCommsController.awaitHandshake();

    serial::serialEmpty(serialFileDescriptor);

    nextPingTime = ros::Time::now() + ros::Duration(MAX_DELAY_SEC);

    while (ros::ok())
    {
        ros::spinOnce();

        controlLoop(rovCommsController, leak_pub, motion_pub, depth_pub);
    } // while ros::ok
}

int lastNumberOfBytesRead = 0;

void softReset(RovCommsController& rov_comms_controller)
{
    rov_comms_controller.reset();
    rov_comms_controller.awaitHandshake();
    mode                  = Modes::COPI;
    nextPingTime          = ros::Time::now() + ros::Duration(MAX_DELAY_SEC);
    lastNumberOfBytesRead = 0;
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
            for (const auto throttle : ros_data::motorThrottles)
            {
                rov_comms_controller.send(throttle);
            }

            rov_comms_controller.send(ros_data::switchControl);

            // Send Servo Angles
            for (const auto angle : ros_data::servoAngles)
            {
                rov_comms_controller.send(angle);
            }

            ROS_DEBUG("Entering mode: CIPO");
            mode         = Modes::CIPO;
            nextPingTime = ros::Time::now() + ros::Duration(MAX_DELAY_SEC);

            break;
        }

    case Modes::CIPO:
        {
            if (ros::Time::now() >= nextPingTime)
            {
                ROS_DEBUG("Timeout: %f",
                          (ros::Time::now() - nextPingTime).toSec());
                ROS_WARN("Timeout when communicating with arduino (in CIPO), "
                         "resetting.");
                softReset(rov_comms_controller);
            }

            int bytesAvailable = rov_comms_controller.tryReadingData();
            if (bytesAvailable == -1)
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
            else if (bytesAvailable != lastNumberOfBytesRead)
            {
                lastNumberOfBytesRead = bytesAvailable;
                nextPingTime = ros::Time::now() + ros::Duration(MAX_DELAY_SEC);
            }
            break;
        }

    case Modes::CIPO_SUCCESSFUL:
        {
            ROS_DEBUG("MODE: CIPO_SUCCESSFUL");

            rov_comms_controller.resetReadBuffer();
            leakMessage.leaks = rov_comms_controller.popReadBuffer();
            leak_pub.publish(leakMessage);

            std::uint8_t* data = rov_comms_controller.popReadBuffer(14);
            motion_pub.publish(processMpu6050Data(data));

            float depth = rov_comms_controller.popReadBuffer<float>();
            std_msgs::Float32 depthMessage;
            depthMessage.data = depth;
            depth_pub.publish(depthMessage);

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
