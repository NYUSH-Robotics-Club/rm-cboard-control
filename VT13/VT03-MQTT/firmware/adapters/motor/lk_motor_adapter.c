/* 瓴控适配器：支持工具中预先开启的 0x280 四电机广播电流模式。 */
#include "motor_adapter.h"
#include "vendor_control.h"
#include "bsp_can.h"
#include "lk_motor_protocol.h"
#include <stddef.h>
#include <string.h>

#define VENDOR_MOTOR_CAPACITY 16U
#define LK_BROADCAST_ID 0x280U

typedef struct {
    bool configured;
    bool pending;
    int16_t command;
    VendorControl control;
} LkContext;

static LkContext s_contexts[VENDOR_MOTOR_CAPACITY];

static BspCanChannel bsp_channel(CAN_Channel_t channel)
{
    return channel == CAN_CHANNEL_2 ? BSP_CAN_CHANNEL_2 : BSP_CAN_CHANNEL_1;
}

static RobotStatus lk_validate(const MotorConfig_t *config)
{
    if (!config || config->vendor != MOTOR_VENDOR_LK ||
        config->type != MOTOR_TYPE_VENDOR_DEFINED ||
        config->motor_id >= VENDOR_MOTOR_CAPACITY ||
        (config->direction != 1 && config->direction != -1) ||
        config->can_channel >= CAN_CHANNEL_COUNT ||
        config->can_tx_id != LK_BROADCAST_ID ||
        config->can_rx_id < 0x141U || config->can_rx_id > 0x144U ||
        config->tx_slot > 3U ||
        config->can_rx_id != (uint16_t)(0x141U + config->tx_slot) ||
        config->protocol.lk.command_limit <= 0 ||
        config->protocol.lk.encoder_counts_per_rev == 0U) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    return ROBOT_STATUS_OK;
}

static RobotStatus lk_init(const RobotConfig_t *robot)
{
    if (!robot || (robot->total_motor_count > 0U && !robot->motor_configs)) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    memset(s_contexts, 0, sizeof(s_contexts));
    bool used_slots[CAN_CHANNEL_COUNT][4] = {{false}};
    for (uint8_t index = 0U; index < robot->total_motor_count; ++index) {
        const MotorConfig_t *config = &robot->motor_configs[index];
        if (config->vendor != MOTOR_VENDOR_LK) continue;
        RobotStatus status = lk_validate(config);
        if (status != ROBOT_STATUS_OK ||
            used_slots[config->can_channel][config->tx_slot]) {
            return ROBOT_STATUS_INVALID_ARGUMENT;
        }
        used_slots[config->can_channel][config->tx_slot] = true;
        LkContext *context = &s_contexts[config->motor_id];
        context->configured = true;
        VendorControl_Init(&context->control, config);
    }
    return ROBOT_STATUS_OK;
}

static RobotStatus lk_command_current(uint8_t motor_id, int16_t current)
{
    if (motor_id >= VENDOR_MOTOR_CAPACITY || !s_contexts[motor_id].configured) {
        return ROBOT_STATUS_NOT_READY;
    }
    LkContext *context = &s_contexts[motor_id];
    const int16_t limit = context->control.config->protocol.lk.command_limit;
    if (current > limit) current = limit;
    if (current < -limit) current = (int16_t)-limit;
    context->command = current;
    context->pending = true;
    return ROBOT_STATUS_OK;
}

static RobotStatus lk_stop(uint8_t motor_id)
{
    return lk_command_current(motor_id, 0);
}

static RobotStatus lk_compute_current(uint8_t motor_id, MotorControlMode_e mode,
                                      float setpoint, int16_t application_current,
                                      int16_t *output)
{
    if (motor_id >= VENDOR_MOTOR_CAPACITY || !s_contexts[motor_id].configured) {
        return ROBOT_STATUS_NOT_READY;
    }
    LkContext *context = &s_contexts[motor_id];
    return VendorControl_Compute(&context->control, mode, setpoint,
                                 application_current,
                                 context->control.config->protocol.lk.command_limit,
                                 output);
}

static RobotStatus lk_snapshot(uint8_t motor_id, MotorSnapshot *snapshot)
{
    if (motor_id >= VENDOR_MOTOR_CAPACITY || !snapshot ||
        !s_contexts[motor_id].configured) return ROBOT_STATUS_INVALID_ARGUMENT;
    *snapshot = s_contexts[motor_id].control.snapshot;
    return snapshot->feedback_valid ? ROBOT_STATUS_OK : ROBOT_STATUS_NOT_READY;
}

static void lk_on_can_frame(const CanRxFrame *frame)
{
    if (!frame || frame->dlc != 8U) return;
    for (uint8_t motor_id = 0U; motor_id < VENDOR_MOTOR_CAPACITY; ++motor_id) {
        LkContext *context = &s_contexts[motor_id];
        const MotorConfig_t *config = context->control.config;
        if (!context->configured || !config || frame->channel != config->can_channel ||
            frame->std_id != config->can_rx_id) continue;
        LkMotorFeedback feedback;
        if (!LkMotorProtocol_DecodeFeedback(frame->data, &feedback)) return;
        context->control.snapshot.feedback_valid = true;
        context->control.snapshot.position =
            (float)feedback.encoder * 360.0f /
            (float)config->protocol.lk.encoder_counts_per_rev;
        context->control.snapshot.speed = (float)feedback.speed_deg_s;
        context->control.snapshot.torque_or_current = (float)feedback.current;
        context->control.snapshot.feedback_timestamp_ms = frame->tick_ms;
        return;
    }
}

static void lk_reset_control(uint8_t motor_id)
{
    if (motor_id < VENDOR_MOTOR_CAPACITY && s_contexts[motor_id].configured) {
        VendorControl_Reset(&s_contexts[motor_id].control);
    }
}

static void lk_flush_channel(CAN_Channel_t channel)
{
    int16_t commands[4] = {0};
    bool has_motor = false;
    bool has_pending = false;
    for (uint8_t motor_id = 0U; motor_id < VENDOR_MOTOR_CAPACITY; ++motor_id) {
        LkContext *context = &s_contexts[motor_id];
        const MotorConfig_t *config = context->control.config;
        if (!context->configured || !config || config->can_channel != channel) continue;
        has_motor = true;
        commands[config->tx_slot] = context->command;
        has_pending = has_pending || context->pending;
    }
    if (!has_motor || !has_pending) return;

    uint8_t data[8];
    if (!LkMotorProtocol_BuildBroadcastCurrent(commands, data) ||
        !BspCan_Write(bsp_channel(channel), LK_BROADCAST_ID, data, sizeof(data))) return;

    for (uint8_t motor_id = 0U; motor_id < VENDOR_MOTOR_CAPACITY; ++motor_id) {
        const MotorConfig_t *config = s_contexts[motor_id].control.config;
        if (s_contexts[motor_id].configured && config && config->can_channel == channel) {
            s_contexts[motor_id].pending = false;
        }
    }
}

static void lk_flush(void)
{
    lk_flush_channel(CAN_CHANNEL_1);
    lk_flush_channel(CAN_CHANNEL_2);
}

const MotorAdapterOps *LkMotorAdapter_Get(void)
{
    static const MotorAdapterOps operations = {
        .vendor = MOTOR_VENDOR_LK,
        .name = "LK broadcast-current adapter",
        .implemented = true,
        .init = lk_init,
        .validate = lk_validate,
        .command_current = lk_command_current,
        .stop = lk_stop,
        .compute_current = lk_compute_current,
        .snapshot = lk_snapshot,
        .on_can_frame = lk_on_can_frame,
        .reset_control = lk_reset_control,
        .flush = lk_flush
    };
    return &operations;
}
