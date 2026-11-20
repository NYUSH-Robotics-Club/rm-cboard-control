/*
 * 从电机服务快照判断失联，并用非阻塞时序轮流输出声光报码。
 * 硬件ID由配置明确给出，不从未知厂商CAN帧推测；反馈恢复后取消报警。
 */
#include "motor_offline_alarm.h"
#include "motor_service.h"
#include "robot_config.h"
#include "bsp_alarm.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define FEEDBACK_TIMEOUT_MS 100U
#define PULSE_MS 100U
#define MIN_GROUP_MS 1000U
#define SEPARATOR_MS 400U
#define NO_MOTOR 0xFFU

static const RobotConfig_t *s_robot;
static MotorOfflineAlarmDiagnostics s_diag;
static bool s_initialized;
static bool s_active;
static uint8_t s_next_index;
static uint32_t s_group_start;
static uint32_t s_separator_start;
static uint32_t s_last_call;

void MotorOfflineAlarm_Init(const RobotConfig_t *robot)
{
    s_robot = robot;
    s_initialized = true;
    s_active = false;
    s_next_index = 0U;
    memset(&s_diag, 0, sizeof(s_diag));
    s_diag.active_motor_id = NO_MOTOR;
    if (!robot || (robot->total_motor_count && !robot->motor_configs) ||
        robot->total_motor_count > 16U) {
        s_diag.config_invalid = true;
        return;
    }
    uint16_t ids = 0U;
    for (uint8_t i = 0; i < robot->total_motor_count; ++i) {
        uint8_t id = robot->motor_configs[i].motor_id;
        if (id >= 16U || (ids & (1U << id))) {
            s_diag.config_invalid = true;
            return;
        }
        ids |= (uint16_t)(1U << id);
    }
}

static uint8_t bus_number(CAN_Channel_t channel)
{
    if (channel == CAN_CHANNEL_1) return 1U;
    if (channel == CAN_CHANNEL_2) return 2U;
    return 0U;
}

static void refresh_offline(uint32_t now_ms)
{
    s_diag.offline_mask = 0U;
    s_diag.unmapped_mask = 0U;
    for (uint8_t i = 0; i < s_robot->total_motor_count; ++i) {
        const MotorConfig_t *motor = &s_robot->motor_configs[i];
        uint16_t bit = (uint16_t)(1U << motor->motor_id);
        MotorSnapshot snapshot = {0};
        if (MotorService_GetSnapshot(motor->motor_id, &snapshot) != ROBOT_STATUS_OK ||
            !snapshot.initialized || !snapshot.feedback_valid ||
            now_ms - snapshot.feedback_timestamp_ms > FEEDBACK_TIMEOUT_MS) {
            s_diag.offline_mask |= bit;
        }
        if (motor->offline_alarm_id == 0U || motor->offline_alarm_id > 16U ||
            bus_number(motor->can_channel) == 0U) s_diag.unmapped_mask |= bit;
    }
}

static void start_group(uint32_t now_ms)
{
    for (uint8_t n = 0; n < s_robot->total_motor_count; ++n) {
        uint8_t index = s_next_index;
        s_next_index = (uint8_t)((index + 1U) % s_robot->total_motor_count);
        const MotorConfig_t *motor = &s_robot->motor_configs[index];
        if (!(s_diag.offline_mask & (1U << motor->motor_id))) continue;
        s_diag.active_motor_id = motor->motor_id;
        s_diag.bus_number = bus_number(motor->can_channel);
        s_diag.hardware_id = (s_diag.unmapped_mask & (1U << motor->motor_id)) ?
                            0U : motor->offline_alarm_id;
        s_diag.blue_separator = false;
        s_group_start = now_ms;
        s_active = true;
        return;
    }
}

void MotorOfflineAlarm_Task(uint32_t now_ms)
{
    if (!s_initialized) MotorOfflineAlarm_Init(RobotConfig_Get());
    if (s_diag.config_invalid) {
        BspAlarm_Set(true, false, false, false);
        return;
    }
    refresh_offline(now_ms);
    if (!s_diag.offline_mask) {
        s_active = false;
        s_diag.active_motor_id = NO_MOTOR;
        s_diag.bus_number = s_diag.hardware_id = s_diag.blue_separator = 0U;
        BspAlarm_Set(false, true, false, false);
        s_last_call = now_ms;
        return;
    }
    if (!s_active) start_group(now_ms);
    else if (!s_diag.blue_separator &&
             (!(s_diag.offline_mask & (1U << s_diag.active_motor_id)) ||
              now_ms - s_last_call >= PULSE_MS)) {
        /* 已恢复的节点不继续鸣叫；循环卡顿时不压缩补发遗漏的脉冲。 */
        s_diag.blue_separator = true;
        s_separator_start = now_ms;
    }
    s_last_call = now_ms;
    if (s_diag.blue_separator) {
        if (now_ms - s_separator_start < SEPARATOR_MS) {
            BspAlarm_Set(false, false, true, false);
            return;
        }
        start_group(now_ms);
    }
    uint32_t elapsed = now_ms - s_group_start;
    uint32_t red_duration = 2U * PULSE_MS * s_diag.hardware_id;
    uint32_t duration = red_duration > MIN_GROUP_MS ? red_duration : MIN_GROUP_MS;
    if (elapsed >= duration) {
        s_diag.blue_separator = true;
        s_separator_start = now_ms;
        BspAlarm_Set(false, false, true, false);
        return;
    }
    bool red = s_diag.hardware_id == 0U ||
               (elapsed < red_duration && elapsed % (2U * PULSE_MS) < PULSE_MS);
    /* 每组开始先鸣n次，每次100ms，中间100ms静音；蓝灯期间静音。 */
    bool beep = elapsed < 2U * PULSE_MS * s_diag.bus_number &&
                elapsed % (2U * PULSE_MS) < PULSE_MS;
    BspAlarm_Set(red, false, false, beep);
}

const MotorOfflineAlarmDiagnostics *MotorOfflineAlarm_GetDiagnostics(void)
{
    return &s_diag;
}
