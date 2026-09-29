/* Host scheduler seam; the ARM build checks the real kernel and port. */
#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
typedef uint32_t TickType_t;
typedef uint32_t StackType_t;
typedef struct { unsigned unused; } StaticTask_t;
typedef StaticTask_t *TaskHandle_t;
#define configMINIMAL_STACK_SIZE 128U
#define configTICK_RATE_HZ 1000U
#define configSTACK_DEPTH_TYPE uint32_t
#define tskIDLE_PRIORITY 0U
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define taskDISABLE_INTERRUPTS() ((void)0)
#endif
