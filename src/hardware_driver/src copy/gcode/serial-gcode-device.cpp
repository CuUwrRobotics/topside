#include "./serial-gcode-device.hpp"

#include <stdio.h>
#include <unistd.h>

#include "serial.hpp"

SerialGcodeDevice::SerialGcodeDevice(gcodeUpdateData_f updateFunction) {
  this->fd = -1;
  this->state = GRS_INIT;
  this->updateFunc = updateFunction;
}

bool SerialGcodeDevice::init(int baud, const char *device) {
  // if (wiringPiSetup() < 0) { // no longer using wiringPi
  //   ROS_ERROR("Unable to start wiringPi.\n");
  // }
  this->fd = serial::openSerialPort(device, baud);
  if (this->fd < 0) {
    ROS_ERROR(": Unable to open serial device; '%s'\n", strerror(errno));
    return false;
  }
  serial::serialFlush(this->fd);
  return true;
}

std::string SerialGcodeDevice::buildRequestStr(GcodeRequest_t &request) {
  std::string request_str = "";
  request_str += commands::getCmdStringFor(request.cmd);
  for (int i = 0; i < MAX_GCODE_ARGS; i++) {
    if (request.args[i].name == '\0') {
      break;
    }
    request_str += ' ' + this->getArgStringFor(request.args[i]);
  }
  return request_str + '\n';
}

void SerialGcodeDevice::stepStateMachine() {
  uint8_t databyte;
  switch (this->state) {
    case GRS_INIT:
      this->state = GRS_WAITING;
      break;
    case GRS_WAITING:
      if (serial::serialDataAvail(fd)) {
        this->state = GRS_READING_BYTES;
      }
      break;
    case GRS_READING_BYTES:
      while (serial::serialDataAvail(fd)) {
        databyte = serial::serialGetchar(this->fd);
        if (databyte != '\n' && databyte != '\r') {
          if (databyte == '\x06') {
            // Recieved acknowledge, ignore it.
          } else if (databyte == '\x15') {
            ROS_ERROR("Recieved NACK for some previous message!");
          } else {
            // Normal character
            this->rx_buffer += databyte;
          }
        } else {
          // Line ended, ready to interpret
          if (this->rx_buffer.empty()) break;  // Catch any spare newlines
          ROS_DEBUG("GCODE READ: %s", this->rx_buffer.c_str());
          this->updateFunc(this->parseCommand(this->rx_buffer), this->getOnlyResponseCharacters(this->rx_buffer));
          this->rx_buffer = "";
          this->state = GRS_WAITING;
        }
      }
      break;
    default:
      ROS_ERROR(": Code should not have reached this state!");
      this->state = GRS_INIT;
      break;
  }
}

bool SerialGcodeDevice::send(GcodeRequest_t &request) {
  std::string request_str = this->buildRequestStr(request);
  // serialPuts(this->fd, request_str.c_str());
  char *cstr = new char[request_str.length() + 1];
  std::strcpy(cstr, request_str.c_str());
  ssize_t written = ::write(this->fd, cstr, strlen(cstr));
  ROS_DEBUG("GCODE WRITE (%ld): %s", written, cstr);
  delete[] cstr;
  return true;
}

bool SerialGcodeDevice::dump(GcodeRequest_t *request_arr) {
  return true;
}

std::string floatToShortenedStr(float input) {
  std::string str = std::to_string(input);

  // Strip trailing zeros
  for (std::string::iterator it = str.end() - 1; it != str.begin(); it--) {
    if ((*it) == '0' || (*it) == '.') {
      str.erase(it);
    } else {
      break;
    }
  }
  return str;
}

std::string SerialGcodeDevice::getArgStringFor(GcodeArg_t arg) {
  std::string result = "";
  result += arg.name;
  result += floatToShortenedStr(arg.value);
  return result;
}

// Strips all characters from the beginning, up to (and including) the first space
std::string SerialGcodeDevice::getOnlyResponseCharacters(std::string response) {
  // No need for incrementing the iterator since 'erase' handles it
  for (std::string::iterator it = response.begin(); it != response.end();) {
    if ((*it) != ' ' && (*it) != '\0' && (*it) != '\n' && (*it) != '\r') {
      // Remove the command part
      response.erase(it);
    } else if ((*it) == ' ') {
      // Delete the space then leave
      response.erase(it);
      break;
    } else {
      // Reached some sort of string ending, don't erase it
      break;
    }
  }

  // Delete any trailing newline characters
  while ((*response.end()) == '\n' || (*response.end()) == '\r') {
    response.erase(response.end());
  }
  return response;
}

GcodeCommand_t SerialGcodeDevice::parseCommand(std::string response) {
  std::string num_only_cmd;
  if (response[0] != 'M') {
    ROS_WARN("Unexpected GCODE reponse being parsed: '%s'", response.c_str());
  }
  for (std::string::iterator it = response.begin(); it != response.end(); it++) {
    if ((*it) != ' ' && (*it) != '\0' && (*it) != '\n' && (*it) != '\r') {
      if ((*it) >= '0' && (*it) <= '9') {
        num_only_cmd += (*it);
      }
    } else {
      break;
    }
  }
  uint16_t cmd_num = atoi(num_only_cmd.c_str());
  // ROS_INFO("Converting '%s' got %d\n", num_only_cmd.c_str(), cmd_num);

  return commands::getCmdObjFor(cmd_num);
}
