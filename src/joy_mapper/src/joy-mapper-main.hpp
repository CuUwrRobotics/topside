#ifndef JOY_MAPPER_MAIN
#define JOY_MAPPER_MAIN

// typedef enum {
//   MS_STOPPED = 0,
//   MS_REGULAR_MOTION = 1,
//   // MS_ARM_MOTION = 2,
//   MS_CAMERA_MOTION = 3
// } MapperState_t;

typedef struct {
  float updown;
  float leftright; 
} Joystick_t;

typedef bool JoyButton_t;
typedef float JoyTrigger_t;

#endif /* end of include guard: JOY_MAPPER_MAIN */
