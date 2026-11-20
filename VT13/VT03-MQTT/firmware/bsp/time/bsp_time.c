/* 板级时间实现；当前沿用 STM32 HAL 的毫秒时基。 */
#include "bsp_time.h"
#include "main.h"

uint32_t BspTime_NowMs(void)
{
    return HAL_GetTick();
}

uint32_t BspTime_NowUs(void)
{
    return HAL_GetTick() * 1000U;
}

void BspTime_DelayMs(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}
