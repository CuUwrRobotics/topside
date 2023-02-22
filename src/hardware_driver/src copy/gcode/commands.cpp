#include <ros/ros.h>
#include <string.h>

#include "./serial-gcode-device.hpp"

std::string commands::getCmdStringFor(GcodeCommand_t cmd) {
  switch (cmd) {
    case CMD_PING:
      return "M0";
      break;
    case CMD_MOTION_MOTORS:
      return "M2";
      break;
    case CMD_ARM_MOTORS:
      return "M3";
      break;
    case CMD_LEAK:
      return "M4";
      break;
    case CMD_POSITION:
      return "M5";
      break;
    default:
      ROS_ERROR("commands::getCmdStringFor: Unknown command!");
      return "E0";
      break;
  }
}

GcodeCommand_t commands::getCmdObjFor(uint16_t cmd) {
  switch (cmd) {
    case 0:
      return CMD_PING;
      break;
    case 2:
      return CMD_MOTION_MOTORS;
      break;
    case 3:
      return CMD_ARM_MOTORS;
      break;
    case 4:
      return CMD_LEAK;
      break;
    case 5:
      return CMD_POSITION;
      break;
    default:
      ROS_ERROR("commands::getCmdObjFor: Unknown value recieved: %d", cmd);
      return _CMD_NONE;
      break;
  }
}

std::string commands::toString(GcodeCommand_t cmd) {
  switch (cmd) {
    case CMD_PING:
      return "CMD_PING";
      break;
    case CMD_MOTION_MOTORS:
      return "CMD_MOTION_MOTORS";
      break;
    case CMD_ARM_MOTORS:
      return "CMD_ARM_MOTORS";
      break;
    case CMD_LEAK:
      return "CMD_LEAK";
      break;
    case CMD_POSITION:
      return "CMD_POSITION";
      break;
    case _CMD_NONE:
      return "_CMD_NONE";
      break;
    default:
      ROS_ERROR("commands::toString: Unknown value recieved!");
      return "<error_command>";
      break;
  }
}
