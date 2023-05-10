#include <math.h>

#include "mpu6050.hpp"

custom_msgs::EulerMotion processMpu6050Data(uint8_t raw_buffer[])
{
    // printf("raw_buffer: ");
    // for (int i = 0; i < 14; i++)
    // {
    //     printf("%d, ", raw_buffer[i]);
    // }
    // printf("\n");

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

    // Units are 2g per-one; this converts to 1g per one
    accel.x *= 2;
    accel.y *= 2;
    accel.z *= 2;

    Vector3D gravity(
        (2 * (gyro.b * gyro.d - gyro.a * gyro.c)), // Multiply by two to convert to 1g like above
        (2 * (gyro.a * gyro.b + gyro.c * gyro.d)),
        ((gyro.a * gyro.a) - (gyro.b * gyro.b) - (gyro.c * gyro.c) + (gyro.d * gyro.d)));
        const double LOCAL_GRAV = 9.81818; /* Set the local gravity before use https://www.sensorsone.com/local-gravity-calculator/#height*/
        const double LIL_G = 9.80665; /* Gravity For 1g*/
        gravity.x *= LOCAL_GRAV /  LIL_G;
        gravity.y *= LOCAL_GRAV / LIL_G; 
        gravity.z *= LOCAL_GRAV / LIL_G; 

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

    // Fix the physical mapping of the ROV
    // TODO: This should be configurable and not hidden away in this function
    float temp = result.roll;
    result.roll = result.pitch;
    result.pitch = temp;

    result.x *= LOCAL_GRAV;
    result.y *= LOCAL_GRAV;
    result.z *= LOCAL_GRAV;

    return result;
}
