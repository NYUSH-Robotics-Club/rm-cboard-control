/*
 * Owns the periodic control cycle. Keeping dispatch and control in one task
 * preserves the old execution order while RTOS support is introduced safely.
 */
#include "robot_rtos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_subscriptions.h"
#include "bsp_time.h"
#include "can_manager.h"
#include "gyro_data.h"
#include "logger.h"
#include "message_center.h"
#include "motor_offline_alarm.h"

/* HAL_Delay(1) waited for two 1 ms tick edges after each completed cycle. */
#define CONTROL_TASK_DELAY_TICKS (pdMS_TO_TICKS(1U) + 1U)
_Static_assert(configTICK_RATE_HZ == 1000U, "control pacing assumes the HAL 1 kHz tick");
#define CONTROL_TASK_STACK_WORDS 1024U
#define CONTROL_TASK_PRIORITY   (tskIDLE_PRIORITY + 4U)

extern CAN_Manager_t can1_manager;
extern CAN_Manager_t can2_manager;

static StaticTask_t s_control_task_buffer;
static StackType_t s_control_task_stack[CONTROL_TASK_STACK_WORDS];
static StaticTask_t s_idle_task_buffer;
static StackType_t s_idle_task_stack[configMINIMAL_STACK_SIZE];
static SensorData s_sensor_data;

static void log_can_health(uint32_t now_ms)
{
    static uint32_t last_log_ms;
    static uint32_t last_can1_rx;
    static uint32_t last_can2_rx;

    if ((uint32_t)(now_ms - last_log_ms) < 1000U) {
        return;
    }

    last_log_ms = now_ms;
    const uint32_t can1_delta = can1_manager.rx_frames - last_can1_rx;
    const uint32_t can2_delta = can2_manager.rx_frames - last_can2_rx;
    last_can1_rx = can1_manager.rx_frames;
    last_can2_rx = can2_manager.rx_frames;

    LOG_CSV(LOG_TAG_CAN, "1,%u,0x%03X,%u,2,%u,0x%03X,%u",
            can1_manager.rx_frames,
            (unsigned int)can1_manager.last_rx_id,
            (unsigned int)can1_delta,
            can2_manager.rx_frames,
            (unsigned int)can2_manager.last_rx_id,
            (unsigned int)can2_delta);
}

static void control_task(void *argument)
{
    (void)argument;

    for (;;) {
        /* Application timestamps stay on the HAL/BSP clock used before RTOS. */
        const uint32_t now_ms = BspTime_NowMs();

        /* Keep command dispatch, motor refresh, and the alarm on one task. */
        gyro_data_update(&s_sensor_data);
        AppRuntime_Step(now_ms);
        CmdController_Task(now_ms);
        MsgCenter_Dispatch();
        /* Dispatch may publish feedback newer than the cycle-start timestamp. */
        MotorOfflineAlarm_Task(BspTime_NowMs());
        log_can_health(now_ms);

        /* Preserve callback-based ramps; do not replay overdue control cycles. */
        vTaskDelay(CONTROL_TASK_DELAY_TICKS);
    }
}

bool RobotRtos_Start(void)
{
    TaskHandle_t handle = xTaskCreateStatic(control_task,
                                             "control",
                                             CONTROL_TASK_STACK_WORDS,
                                             NULL,
                                             CONTROL_TASK_PRIORITY,
                                             s_control_task_stack,
                                             &s_control_task_buffer);
    if (handle == NULL) {
        return false;
    }

    vTaskStartScheduler();
    return false;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **task_buffer,
                                   StackType_t **stack_buffer,
                                   configSTACK_DEPTH_TYPE *stack_size)
{
    *task_buffer = &s_idle_task_buffer;
    *stack_buffer = s_idle_task_stack;
    *stack_size = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}
