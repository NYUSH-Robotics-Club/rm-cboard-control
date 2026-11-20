/*
 * Defines the platform-neutral IMU state consumed by robot applications.
 * Sensor drivers publish this message after converting device-specific data.
 */
#ifndef SENSOR_MESSAGES_H
#define SENSOR_MESSAGES_H

#include <stdint.h>

typedef struct {
    /* Gimbal IMU: angular rates in rad/s and acceleration in m/s^2. */
    float g_gx;
    float g_gy;
    float g_gz;
    float g_ax;
    float g_ay;
    float g_az;

    /* Gimbal attitude in degrees. */
    float yaw;
    float pitch;
    float roll;
    float yaw_total_angle;
    int32_t yaw_round_count;

    /* Kept while legacy gimbal calculations are being migrated. */
    float absolute_angle;

    /* Chassis IMU: attitude in degrees, rate in rad/s, acceleration in m/s^2. */
    float c_roll;
    float c_pitch;
    float c_yaw;
    float c_gx;
    float c_gy;
    float c_gz;
    float c_ax;
    float c_ay;
    float c_az;
} SensorData;

#endif
