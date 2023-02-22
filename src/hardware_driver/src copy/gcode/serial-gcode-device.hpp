#ifndef ARDUINO_SERIAL_HPP
#define ARDUINO_SERIAL_HPP

#include <ros/ros.h>
#include <stdint.h>
#include <string.h>

#include "./commands.hpp"

typedef enum {
  GRS_INIT = 0,
  GRS_WAITING,
  GRS_READING_BYTES
} GcodeRecieverState_t;

typedef void (*gcodeUpdateData_f)(GcodeCommand_t, std::string);

namespace commands {
std::string getCmdStringFor(GcodeCommand_t cmd);
GcodeCommand_t getCmdObjFor(uint16_t cmd);
std::string toString(GcodeCommand_t cmd);
}  // namespace commands

class SerialGcodeDevice {
 private:
  int fd;
  uint8_t state;
  std::string rx_buffer;
  gcodeUpdateData_f updateFunc;

  // String building
  std::string getArgStringFor(GcodeArg_t arg);
  std::string buildRequestStr(GcodeRequest_t &request);

  // String parsing
  GcodeCommand_t parseCommand(std::string response);
  std::string getOnlyResponseCharacters(std::string response);

 public:
  SerialGcodeDevice(gcodeUpdateData_f updateFunction);
  bool init(int baudrate, const char *device);
  void stepStateMachine();
  bool send(GcodeRequest_t &request);
  bool dump(GcodeRequest_t *request_arr);
};

#endif  // End of include guard for ARDUINO_SERIAL_HPP
