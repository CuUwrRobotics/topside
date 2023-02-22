#ifndef MPU6050_PROCESSOR_HPP
#define MPU6050_PROCESSOR_HPP

#include <custom_msgs/EulerMotion.h>
#include <stdint.h>

typedef struct struct_Vector3D
{
  float x, y, z;
  struct_Vector3D(float xx, float yy, float zz)
      : x(xx), y(yy), z(zz) {}
} Vector3D;

typedef struct struct_Quaternion3D
{
  float a, b, c, d;
  struct_Quaternion3D(float aa, float bb, float cc, float dd)
      : a(aa), b(bb), c(cc), d(dd) {}
} Quaternion3D;

custom_msgs::EulerMotion processMpu6050Data(uint8_t raw_buffer[]);

#endif // End of include guard for MPU6050_PROCESSOR_HPP
