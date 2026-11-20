/* 声光BSP主机测试的GPIO替身，不模拟控制逻辑。 */
#ifndef TEST_ALARM_MAIN_H
#define TEST_ALARM_MAIN_H
#include <stdint.h>
#define GPIOH ((void *)1)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_PIN_12 (1U << 12)
typedef enum {GPIO_PIN_RESET, GPIO_PIN_SET} GPIO_PinState;
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state);
#endif
