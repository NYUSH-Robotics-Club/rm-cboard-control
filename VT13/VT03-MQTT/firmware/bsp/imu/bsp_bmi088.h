/*
 * 提供 BMI088 驱动需要的 SPI、片选和短延时操作。
 * 传感器寄存器和姿态算法不属于本文件。
 */
#ifndef BSP_BMI088_H
#define BSP_BMI088_H

#include <stdint.h>

#define BMI088_USE_SPI

void BMI088_GPIO_init(void);
void BMI088_com_init(void);
void BMI088_delay_ms(uint16_t ms);
void BMI088_delay_us(uint16_t us);
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);
uint8_t BMI088_ACCEL_NS_READ(void);
uint8_t BMI088_GYRO_NS_READ(void);
uint8_t BMI088_read_write_byte(uint8_t value);

#endif
