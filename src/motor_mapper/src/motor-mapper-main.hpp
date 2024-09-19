#pragma once

#include "acceleration.hpp"

enum class MotorType_t
{
    MT_UNKNOWN = 0,
    MT_ESC,
    /*MT_H_BRIDGE,*/
};

struct Vector3
{
    float x;
    float y;
    float z;
};

struct R3_t
{
    float roll;
    float pitch;
    float yaw;
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

