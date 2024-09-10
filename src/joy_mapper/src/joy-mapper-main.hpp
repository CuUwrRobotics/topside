#ifndef JOY_MAPPER_MAIN
#define JOY_MAPPER_MAIN

// typedef enum {
//   MS_STOPPED = 0,
//   MS_REGULAR_MOTION = 1,
//   // MS_ARM_MOTION = 2,
//   MS_CAMERA_MOTION = 3
// } MapperState_t;

struct Joystick_t
{
    float updown;
    float leftright;
};

using JoyButton_t  = bool;
using JoyTrigger_t = float;

#endif /* end of include guard: JOY_MAPPER_MAIN */
