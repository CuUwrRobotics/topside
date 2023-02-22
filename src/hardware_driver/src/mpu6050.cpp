#include <math.h>

#include "mpu6050.hpp"

custom_msgs::EulerMotion processMpu6050Data(uint8_t raw_buffer[])
{

  // Extract the 16-bit signed integers and divide by 2**14 to get a value on
  // range [-1, 1]
  Vector3D accel(
      float(int16_t((raw_buffer[8] << 8) | (raw_buffer[9]))) / float(0x1 << 14),
      float(int16_t((raw_buffer[10] << 8) | (raw_buffer[11]))) / float(0x1 << 14),
      float(int16_t((raw_buffer[12] << 8) | (raw_buffer[13]))) / float(0x1 << 14));
  Quaternion3D gyro(
      float(int16_t((raw_buffer[0] << 8) | (raw_buffer[1]))) / float(0x1 << 14),
      float(int16_t((raw_buffer[2] << 8) | (raw_buffer[3]))) / float(0x1 << 14),
      float(int16_t((raw_buffer[4] << 8) | (raw_buffer[5]))) / float(0x1 << 14),
      float(int16_t((raw_buffer[6] << 8) | (raw_buffer[7]))) / float(0x1 << 14));

  Vector3D gravity(
      (gyro.b * gyro.d - gyro.a * gyro.c),
      (gyro.a * gyro.b + gyro.c * gyro.d),
      ((gyro.a * gyro.a) - (gyro.b * gyro.b) - (gyro.c * gyro.c) + (gyro.d * gyro.d)) / 2);

  Vector3D acceleration_without_gravity(
      accel.x - gravity.x,
      accel.y - gravity.y,
      accel.z - gravity.z);

  custom_msgs::EulerMotion result;
  result.x = acceleration_without_gravity.x;
  result.y = acceleration_without_gravity.y;
  result.z = acceleration_without_gravity.z;
  result.roll = std::atan(gravity.y / std::sqrt(gravity.x * gravity.x + gravity.z * gravity.z));
  result.pitch = std::atan(gravity.x / std::sqrt(gravity.y * gravity.y + gravity.z * gravity.z));
  result.yaw = std::atan2(2 * (gyro.b * gyro.c - gyro.a * gyro.d), 2 * (gyro.a * gyro.a - gyro.b * gyro.b) - 1);
  // result.roll = gravity.x;
  // result.pitch = gravity.y;
  // result.yaw = gravity.z;

  return result;
}
