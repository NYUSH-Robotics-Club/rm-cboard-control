/* 仅替代现有非阻塞蜂鸣器接口，记录调用。 */
#ifndef TEST_ALARM_BUZZER_H
#define TEST_ALARM_BUZZER_H
#include <stdint.h>
void Buzzer_Start(uint32_t frequency);
void Buzzer_Stop(void);
#endif
