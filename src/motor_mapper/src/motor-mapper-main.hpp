#ifndef MOTOR_MAPPER_MAIN
#define MOTOR_MAPPER_MAIN

#include "acceleration.hpp"

typedef enum {
  MT_UNKNOWN = 0,
  MT_ESC,
  /*MT_H_BRIDGE,*/
} MotorType_t;

typedef struct {
  float x, y, z;
} V3_t;

typedef struct {
  float r, p, y;
} R3_t;

typedef struct {
  float power_limit;
  V3_t dir;
  R3_t angle;
  float set_zero, set_delta;
  // MotorType_t type;
  acceleration::Accelerator* accelerator;
} Motor_t;

#endif /* end of include guard: MOTOR_MAPPER_MAIN */
