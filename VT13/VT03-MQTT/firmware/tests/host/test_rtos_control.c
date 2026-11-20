/* Run the real control task and alarm across a tick during message dispatch. */
#include "robot_rtos.h"
#include "task.h"
#include "can_manager.h"
#include "gyro_data.h"
#include "motor_service.h"
#include "motor_offline_alarm.h"
#include "bsp_alarm.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>

CAN_Manager_t can1_manager, can2_manager;
static const MotorConfig_t motor = {
    .motor_id = 0, .offline_alarm_id = 1, .can_channel = CAN_CHANNEL_1
};
static const RobotConfig_t robot = {.motor_configs = &motor, .total_motor_count = 1};
static MotorSnapshot snapshot;
static uint32_t clock_ms;
static uint32_t dispatch_duration_ms = 1U;
static bool stale, create_failure, green_output, dispatched, dashboard_called;
static void (*task_entry)(void *);
static void *task_argument;
static jmp_buf cycle_complete;

const RobotConfig_t *RobotConfig_Get(void) { return &robot; }
RobotStatus MotorService_GetSnapshot(uint8_t id, MotorSnapshot *result)
{
    assert(id == 0);
    *result = snapshot;
    return ROBOT_STATUS_OK;
}
void BspAlarm_Set(bool red, bool green, bool blue, bool beep)
{
    (void)red; (void)blue; (void)beep;
    green_output = green;
}
uint32_t BspTime_NowMs(void) { return clock_ms; }
void gyro_data_update(SensorData *data) { (void)data; }
void AppRuntime_Step(uint32_t now_ms) { (void)now_ms; }
void CmdController_Task(uint32_t now_ms) { (void)now_ms; }
void MsgCenter_Dispatch(void)
{
    dispatched=true;
    clock_ms += dispatch_duration_ms;
    snapshot = (MotorSnapshot){
        .initialized = true, .feedback_valid = true,
        .feedback_timestamp_ms = clock_ms - (stale ? 101U : 0U)
    };
}
void Dashboard_Task(uint32_t now_ms) {
    assert(dispatched && now_ms == clock_ms);
    dashboard_called = true;
}
TaskHandle_t xTaskCreateStatic(void (*entry)(void *), const char *name,
                              uint32_t depth, void *argument, unsigned priority,
                              StackType_t *stack, StaticTask_t *buffer)
{
    (void)name; (void)priority;
    assert(depth > 0 && stack && buffer);
    task_entry = entry;
    task_argument = argument;
    return create_failure ? NULL : buffer;
}
TickType_t xTaskGetTickCount(void) { return 0; }
void vTaskStartScheduler(void) { task_entry(task_argument); }
void vTaskDelay(TickType_t delay)
{
    assert(dashboard_called);
    assert(delay == 2U); /* Match HAL_Delay(1), including its extra tick. */
    longjmp(cycle_complete, 1);
}
static void run_cycle(uint32_t start_ms, bool feedback_stale)
{
    dispatched=dashboard_called=false;
    clock_ms = start_ms;
    stale = feedback_stale;
    MotorOfflineAlarm_Init(&robot);
    if (setjmp(cycle_complete) == 0) {
        (void)RobotRtos_Start();
        assert(!"scheduler returned without a control cycle");
    }
    assert(MotorOfflineAlarm_GetDiagnostics()->offline_mask == (stale ? 1U : 0U));
    assert(green_output == !stale);
}
int main(void)
{
    run_cycle(100U, false);
    run_cycle(UINT32_MAX, false);
    run_cycle(100U, true);
    dispatch_duration_ms = 30U;
    run_cycle(100U, false); /* An overrun still yields; no catch-up burst. */
    create_failure = true;
    assert(!RobotRtos_Start());
    puts("RTOS control: PASS (dispatch tick, wrap, stale feedback, creation failure)");
}
