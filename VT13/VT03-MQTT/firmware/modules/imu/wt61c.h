/*
 * 声明 WT61C 数据解析和可选配置命令。
 * 旧 UART 句柄参数仅为兼容冻结的 main.c；新代码不应继续扩大这种依赖。
 */

#ifndef WT61C_H
#define WT61C_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WT61C sensor data structure
 */
typedef struct {
    float ax, ay, az;       // Acceleration (m/s^2)
    float gx, gy, gz;       // Gyroscope (deg/s)
    float roll, pitch, yaw; // Euler angles (degrees)
    float temperature;      // Temperature (Celsius)
    uint32_t last_update_ms; // Timestamp of last update
} WT61C_Data;

/**
 * @brief Initialize WT61C driver
 * @param huart UART handle connected to WT61C
 */
void WT61C_Init(UART_HandleTypeDef *huart);

/**
 * @brief Process incoming bytes from WT61C sensor
 * @param data Pointer to received data buffer
 * @param len Number of bytes received
 */
void WT61C_ProcessBytes(const uint8_t *data, uint16_t len);

/**
 * @brief Get pointer to latest sensor data
 * @return Pointer to WT61C_Data structure
 */
const WT61C_Data* WT61C_GetData(void);

/* Optional configuration functions */

/**
 * @brief Unlock sensor for configuration
 * @param huart UART handle
 * @return HAL status
 */
HAL_StatusTypeDef WT61C_Unlock(UART_HandleTypeDef *huart);

/**
 * @brief Set sensor data return rate
 * @param huart UART handle
 * @param rateEnum Rate value (0x06=10Hz, 0x07=20Hz, 0x08=50Hz, 0x09=100Hz, 0x0B=200Hz)
 * @return HAL status
 */
HAL_StatusTypeDef WT61C_SetReturnRate(UART_HandleTypeDef *huart, uint8_t rateEnum);

/**
 * @brief Save configuration to sensor flash
 * @param huart UART handle
 * @return HAL status
 */
HAL_StatusTypeDef WT61C_Save(UART_HandleTypeDef *huart);

/**
 * @brief Callback function for new data (weak, can be overridden by user)
 * @param d Pointer to updated sensor data
 */
void WT61C_OnNewData(const WT61C_Data *d);

#ifdef __cplusplus
}
#endif

#endif /* WT61C_H */
