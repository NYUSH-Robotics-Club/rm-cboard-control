/*
 * 更新 IMU 、执行校准并发布统一传感器消息。
 * 应用只需依赖 core/contracts 中的 SensorData。
 */
#ifndef GYRO_DATA_H

#define GYRO_DATA_H


#include <stdint.h>
#include "sensor_messages.h"

#define Initial_Tick 6300.0f// the tick when gimbal pointing forward
#define Max_Tick  8192.0f
#define delta_t 0.005f//5ms


void gyro_data_init(void);
void gyro_data_update(SensorData *sensor_data);
float gimbal_absolute_angle(SensorData* sensor_data);

// 陀螺仪零偏校准
void gyro_calibrate(void);

// 校准期间的回调函数类型（在每次采样间隔时调用，用于保持云台位置等）
typedef void (*GyroCalibCallback_t)(void);

// 设置校准回调函数
void gyro_calibrate_set_callback(GyroCalibCallback_t callback);

#endif
