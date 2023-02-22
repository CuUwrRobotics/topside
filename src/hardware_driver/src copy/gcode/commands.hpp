#ifndef COMMANDS_HPP
#define COMMANDS_HPP

#define MAX_GCODE_ARGS 8

typedef enum {
  _CMD_NONE = -1,
  CMD_PING,
  CMD_MOTION_MOTORS,
  CMD_ARM_MOTORS,
  CMD_LEAK,
  CMD_POSITION,
} GcodeCommand_t;

typedef struct {
  char name;
  float value;
} GcodeArg_t;

typedef struct {
  GcodeCommand_t cmd;
  GcodeArg_t args[MAX_GCODE_ARGS];
} GcodeRequest_t;

#endif  // End of include guard for COMMANDS_HPP
