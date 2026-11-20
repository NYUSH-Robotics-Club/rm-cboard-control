/* 核对实装BSP使用的灯脚和PWM切换次数，防止每轮重启蜂鸣器。 */
#include "bsp_alarm.h"
#include "main.h"
#include <assert.h>
#include <stdio.h>
static uint16_t pins;
static unsigned starts,stops;
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state) {
 assert(port==GPIOH);
 assert(pin==GPIO_PIN_10 || pin==GPIO_PIN_11 || pin==GPIO_PIN_12);
 if(state==GPIO_PIN_SET) pins|=pin;else pins&=(uint16_t)~pin;
}
void Buzzer_Start(uint32_t frequency) { assert(frequency==2000U);++starts; }
void Buzzer_Stop(void) { ++stops; }
int main(void) {
 BspAlarm_Set(false,true,false,false);
 assert(pins==GPIO_PIN_11 && stops==1 && starts==0);
 for(unsigned i=0;i<100;i++) BspAlarm_Set(true,false,false,true);
 assert(pins==GPIO_PIN_12 && starts==1 && stops==1);
 BspAlarm_Set(false,false,true,false);
 assert(pins==GPIO_PIN_10 && stops==2);
 BspAlarm_Set(false,true,false,false);
 assert(pins==GPIO_PIN_11 && stops==2);
 puts("alarm BSP: PASS (RGB pins, sound transitions)");
}
