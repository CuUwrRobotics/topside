// System includes
#include <stdint.h>

#include <cassert>
#include <string>

// ROS includes
// #include <custom_msgs/MotorControls.h>
#include <ros/ros.h>

// Internal inclues
#include "gcode/serial-gcode-device.hpp"
#include "hardware-driver-main.hpp"
#include "ros-callbacks.hpp"

#define NOW() (ros::Time::now())
// Creates a delay in seconds
#define CREATE_DELAY(a) (ros::Time::now() + ros::Duration(a))

void dataRecieved(GcodeCommand_t type, std::string data);

SerialGcodeDevice arduino = SerialGcodeDevice(dataRecieved);

custom_msgs::MotorControls ros_data::motor_controls_msg;

GcodeRequest_t request_list[REQUEST_LIST_LENGTH];

void updateRequestList();
void prepareRequestList();

ros::Time last_ping_time;

int main(int argc, char* argv[]) {
  ROS_INFO("Hardware Driver beginning\n");

  // ROS Initialization
  ros::init(argc, argv, "hardware_driver");
  ros::NodeHandle node;
  ros::Subscriber motor_sub = node.subscribe<custom_msgs::MotorControls>("hardware/main_motors", 1, &motionMotorSubCallback);

  std::string port = "/dev/ttyUSB0";
  
  printf("%d args: ", argc);
  
  for (uint8_t i = 0; i < argc; i++) {
  	printf("'%s', ", argv[i]);
  }
  printf("\n");

  if (argc > 1) {
    for (uint8_t arg = 0; arg < argc; arg++) {
      if (strcmp(argv[arg], "-d") == 0 || strcmp(argv[arg], "-D") == 0) {
        ROS_INFO("-d specified, enabling debug-level logging.");
        if (!ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
          ROS_ERROR(": Debug logging could not be enabled.");
        }
      } else if (strcmp(argv[arg], "-p") == 0) {
        if (argc < arg + 2) {
          ROS_WARN("-p specified, but no port was given. Continuing with default port.");
        } else {
          port = argv[arg + 1];
        }
      }
    }
  }

  // Arduino Connection
  ROS_INFO("Attempting to open Arduino using port '%s'\n", port.c_str());
  if (!arduino.init(115200, port.c_str())) {
    ROS_ERROR(": Initializing Arduino serial port failed. Exiting.\n");
    exit(EXIT_FAILURE);
  }

  {
    // Check if arduino is present before booting
    GcodeRequest_t test_request;
    test_request.cmd = CMD_PING;
    for (int i = 0; i < MAX_GCODE_ARGS; i++) {
      test_request.args[i].name = '\0';
      // test_request.args[i].value = (float)i + ((float)i / 3);
    }

    last_ping_time = NOW();

    ros::Time start = NOW(), end = CREATE_DELAY(5);
    ros::Time timer;
    bool safely_booted = false;
    while (NOW() < end) {
      arduino.send(test_request);
      timer = CREATE_DELAY(0.1);
      while (NOW() < timer) {
        arduino.stepStateMachine();
      }
      if (last_ping_time > start) {
        safely_booted = true;
        break;
      }
    }
    if (!safely_booted) {
      ROS_ERROR(": Did not get echo response within 5 seconds of booting. Exiting just in case.");
      exit(EXIT_FAILURE);
    }
  }

  ROS_INFO("Hardware Driver successfully initialized, beginning main loop.\n");

  request_list[RL_MOTION_MOTORS].cmd = CMD_MOTION_MOTORS;
  request_list[RL_ARM_MOTORS].cmd = CMD_ARM_MOTORS;
  request_list[RL_LEAK].cmd = CMD_LEAK;
  request_list[RL_POSITION].cmd = CMD_POSITION;

  ros::Time end = CREATE_DELAY(2);

  prepareRequestList();
  while (ros::ok()) {
    ros::spinOnce();
    // Dump data 10 times per second
    updateRequestList();
    arduino.send(request_list[RL_MOTION_MOTORS]);
    while (NOW() < end) {
      // This function needs to be run constantly in case there
      // is a delay in the serial stream
      arduino.stepStateMachine();
    }
    end = CREATE_DELAY(0.1);
  }

  return 0;
}

void dataRecieved(GcodeCommand_t type, std::string data) {
  switch (type) {
    case CMD_PING:
      last_ping_time = NOW();
      break;
    case CMD_LEAK:
      ROS_WARN("TODO (leak)");
      break;
    case CMD_POSITION:
      ROS_WARN("TODO (position)");
      break;
    case _CMD_NONE:
      ROS_WARN("Got error reponse. Data: %s", data.c_str());
      // Ignore
      break;
    default:
      ROS_ERROR(": Recieved invalid or unknown GCODE response.");
      break;
  }
  if (type == CMD_PING) {
  }
}

void prepareRequestList() {
#define _MOTION_MOTORS_COUNT (sizeof(ros_data::motor_controls_msg.motor_throttles) / sizeof(ros_data::motor_controls_msg.motor_throttles[0]))
  assert(_MOTION_MOTORS_COUNT < MAX_GCODE_ARGS);
  for (uint8_t i = 0; i < _MOTION_MOTORS_COUNT; i++) {
    request_list[RL_MOTION_MOTORS].args[i].name = 'A' + i;
  }
}

void updateRequestList() {
  for (uint8_t i = 0; i < _MOTION_MOTORS_COUNT; i++) {
    request_list[RL_MOTION_MOTORS].args[i].value = ros_data::motor_controls_msg.motor_throttles[i];
  }
}
