/*
 * 驱动现有C板PH12红/PH11绿/PH10蓝与TIM4蜂鸣器。
 * 报警独占运行阶段的灯和蜂鸣器，启动校准仍使用原流程。
 */
#include "bsp_alarm.h"
#include "main.h"
#include "buzzer.h"

/* 冻结的buzzer.c已导出这两个非阻塞函数，旧头文件未声明，由BSP适配。 */
extern void Buzzer_Start(uint32_t frequency);
extern void Buzzer_Stop(void);

void BspAlarm_Set(bool red, bool green, bool blue, bool beep)
{
    static bool known;
    static bool last_beep;
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, red ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, green ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, blue ? GPIO_PIN_SET : GPIO_PIN_RESET);
    /* 只在声音状态变化时重配PWM，避免每轮重启计数器。 */
    if (!known || beep != last_beep) {
        if (beep) Buzzer_Start(2000U);
        else Buzzer_Stop();
        known = true;
        last_beep = beep;
    }
}
