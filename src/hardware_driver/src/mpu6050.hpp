#ifndef MPU6050_PROCESSOR_HPP
#define MPU6050_PROCESSOR_HPP

#include <cstdint>
#include <custom_msgs/EulerMotion.h>

struct Vector3D
{
    float x;
    float y;
    float z;

    Vector3D(const float xx, const float yy, const float zz)
        : x(xx),
          y(yy),
          z(zz)
    {
    }
};

struct Quaternion3D
{
    float a;
    float b;
    float c;
    float d;

    Quaternion3D(const float aa, const float bb, const float cc, const float dd)
        : a(aa),
          b(bb),
          c(cc),
          d(dd)
    {
    }
};

custom_msgs::EulerMotion processMpu6050Data(std::uint8_t raw_buffer[]);

#endif // End of include guard for MPU6050_PROCESSOR_HPP
