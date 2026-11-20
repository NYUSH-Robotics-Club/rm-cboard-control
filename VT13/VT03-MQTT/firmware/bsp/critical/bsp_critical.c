/*
 * 在 STM32 上用总中断开关保护短操作；主机测试时不做处理。
 * 这里不能包围阻塞调用，只适合复制索引或小块固定长度数据。
 */
#include "bsp_critical.h"

#if defined(USE_HAL_DRIVER)
#include "stm32f4xx_hal.h"
#endif

BspCriticalState BspCritical_Enter(void)
{
#if defined(USE_HAL_DRIVER)
    BspCriticalState previous_state = __get_PRIMASK();
    __disable_irq();
    return previous_state;
#else
    return 0U;
#endif
}

void BspCritical_Exit(BspCriticalState previous_state)
{
#if defined(USE_HAL_DRIVER)
    if (previous_state == 0U) {
        __enable_irq();
    }
#else
    (void)previous_state;
#endif
}
