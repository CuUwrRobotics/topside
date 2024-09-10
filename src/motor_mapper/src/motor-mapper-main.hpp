#ifndef MOTOR_MAPPER_MAIN
#define MOTOR_MAPPER_MAIN

#include "acceleration.hpp"

enum MotorType_t
{
    MT_UNKNOWN = 0,
    MT_ESC,
    /*MT_H_BRIDGE,*/
};

struct Vector3
{
    float x, y, z;
};

struct R3_t
{
    float r, p, y;
};

struct Motor_t
{
    float                      power_limit;
    Vector3                    dir;
    R3_t                       angle;
    float                      set_zero;
    float                      set_delta;
    acceleration::Accelerator* accelerator;
    // MotorType_t type;
};

#endif /* end of include guard: MOTOR_MAPPER_MAIN */
