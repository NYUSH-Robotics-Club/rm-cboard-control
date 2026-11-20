/*
 * 提供上层统一使用的毫秒时间和延时接口。
 * 以后接入 RTOS 时，只需在 BSP 中切换实现，不必修改控制器。
 */
#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

uint32_t BspTime_NowMs(void);
uint32_t BspTime_NowUs(void);
void BspTime_DelayMs(uint32_t delay_ms);

#endif
