/*
 * 提供C板报警灯和蜂鸣器输出；只负责硬件，不解释故障码。
 * GPIO/PWM必须已由现有启动流程初始化，本接口不延时。
 */
#ifndef BSP_ALARM_H
#define BSP_ALARM_H
#include <stdbool.h>

/* RGB为亮灭状态；beep=true输出2kHz提示音，false静音。仅主循环调用。 */
void BspAlarm_Set(bool red, bool green, bool blue, bool beep);
#endif
