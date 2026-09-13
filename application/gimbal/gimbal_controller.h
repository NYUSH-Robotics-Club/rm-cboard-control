/*
 * 声明云台控制和启动对齐接口。
 * 角速度输入为 -1 到 1 的归一化值，返回值是所选电调模式的原始命令刻度。
 * 两轴支持配置中的速度前馈及固定偏置，叠加后仍受协议限幅。
 */
#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "control_messages.h"
#include "sensor_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void last_data(float last_yaw_rate, float last_yaw_target);

/**
 * @brief Pitch control with normalized rate command
 * @param id Configured pitch motor ID
 * @param rate_normalized Normalized pitch rate (-1.0 to 1.0, always responds to joystick)
 * @param sensor_data Sensor data pointer
 * @param disable_yaw_pitch_compensation true强制禁用耦合（自瞄）；false仍须配置显式开启才补偿。
 * @return Native motor command counts; invalid enabled feedforward returns zero
 *         and requires gimbal startup alignment again.
 */

int16_t GimbalController_PitchControl(uint8_t id, float rate_normalized, SensorData* sensor_data, bool disable_yaw_pitch_compensation);

/* 模式明确选择目标来源/限速；普通回中仍属于MANUAL。 */
typedef enum {
    YAW_CONTROL_MANUAL = 0,
    YAW_CONTROL_VISION,
    YAW_CONTROL_SPIN
} YawControlMode;

/*
 * 输入为[-1,1]摇杆；输出为所选电调模式的原始刻度。须先完成启动对齐。
 * 连续跟踪无效返回0并要求重新对齐。速度调试时旁路位置环，仅普通摇杆给RPM。
 */
int16_t GimbalController_YawControlWithCompensation(float rate_normalized,
    SensorData *sensor_data, YawControlMode mode);

/**
 * @brief Target angle correction for yaw (chassis compensation)
 * @param sensor_data Sensor data pointer
 */
void GimbalController_TargetAngleCorrection(SensorData* sensor_data);

/**
 * @brief Initialize gimbal application (message subscriptions and control)
 */
void GimbalApp_Init(void);

/**
 * @brief 等待云台反馈；yaw锁存当前位置，pitch采用配置初始目标（负数则锁存当前位置）。
 * @note No fixed mechanical target is selected during startup. Each configured
 *       axis starts holding only after its first encoder feedback is available.
 *       Timeout: 5 seconds
 */
void Gimbal_WaitForAlignment(void);

/**
 * @brief Calculate and display gimbal tilt angle compensation
 * @note Calculates yaw-pitch coupling compensation when gimbal is tilted
 *       Updates every 100ms, displays via USB CDC and logs to CSV
 *       Compensation model assumes 30cm gimbal height above ground
 */
void GimbalController_CalculateAndDisplayCompensation(void);


#ifdef __cplusplus
}
#endif

#endif // GIMBAL_CONTROLLER_H
