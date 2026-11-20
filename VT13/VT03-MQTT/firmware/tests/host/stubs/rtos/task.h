#ifndef TEST_TASK_H
#define TEST_TASK_H
#include "FreeRTOS.h"
TaskHandle_t xTaskCreateStatic(void (*entry)(void *), const char *name,
                              uint32_t depth, void *argument, unsigned priority,
                              StackType_t *stack, StaticTask_t *buffer);
TickType_t xTaskGetTickCount(void);
void vTaskStartScheduler(void);
void vTaskDelay(TickType_t delay);
#endif
