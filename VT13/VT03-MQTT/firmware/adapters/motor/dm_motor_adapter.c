/* 达妙适配器：统一控制环输出映射为 MIT 力矩帧。 */
#include "motor_adapter.h"
#include "vendor_control.h"
#include "bsp_can.h"
#include "dm_motor_protocol.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define VENDOR_MOTOR_CAPACITY 16U
#define DM_ENABLE_COMMAND 0xFCU
#define DM_DISABLE_COMMAND 0xFDU
#define DM_ENABLE_WARMUP_FRAMES 10U

typedef struct {
    bool configured;
    bool pending;
    bool enabled;
    uint8_t enable_frames_left;
    int16_t command;
    VendorControl control;
} DmContext;

static DmContext s_contexts[VENDOR_MOTOR_CAPACITY];

static BspCanChannel bsp_channel(CAN_Channel_t channel)
{
    return channel == CAN_CHANNEL_2 ? BSP_CAN_CHANNEL_2 : BSP_CAN_CHANNEL_1;
}

static DmMotorLimits protocol_limits(const MotorConfig_t *config)
{
    DmMotorLimits limits = {
        config->protocol.dm.position_min, config->protocol.dm.position_max,
        config->protocol.dm.velocity_min, config->protocol.dm.velocity_max,
        config->protocol.dm.torque_min, config->protocol.dm.torque_max
    };
    return limits;
}

static RobotStatus dm_validate(const MotorConfig_t *config)
{
    if (!config || config->vendor != MOTOR_VENDOR_DM ||
        config->type != MOTOR_TYPE_VENDOR_DEFINED ||
        config->motor_id >= VENDOR_MOTOR_CAPACITY ||
        (config->direction != 1 && config->direction != -1) ||
        config->can_channel >= CAN_CHANNEL_COUNT ||
        config->can_tx_id > 0x7FFU || config->can_rx_id > 0x7FFU) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    const DmMotorLimits limits = protocol_limits(config);
    if (limits.position_min > 0.0f || limits.position_max < 0.0f ||
        limits.velocity_min > 0.0f || limits.velocity_max < 0.0f ||
        limits.torque_min > 0.0f || limits.torque_max < 0.0f) {
        return ROBOT_STATUS_UNSUPPORTED;
    }
    uint8_t probe[8];
    return DmMotorProtocol_EncodeMit(&limits, 0.0f, 0.0f, 0.0f, 0.0f,
                                     0.0f, probe)
               ? ROBOT_STATUS_OK
               : ROBOT_STATUS_UNSUPPORTED;
}

static RobotStatus dm_init(const RobotConfig_t *robot)
{
    if (!robot || (robot->total_motor_count > 0U && !robot->motor_configs)) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    memset(s_contexts, 0, sizeof(s_contexts));
    for (uint8_t index = 0U; index < robot->total_motor_count; ++index) {
        const MotorConfig_t *config = &robot->motor_configs[index];
        if (config->vendor != MOTOR_VENDOR_DM) continue;
        RobotStatus status = dm_validate(config);
        if (status != ROBOT_STATUS_OK) return status;
        for (uint8_t other = 0U; other < index; ++other) {
            const MotorConfig_t *previous = &robot->motor_configs[other];
            if (previous->vendor == MOTOR_VENDOR_DM &&
                previous->can_channel == config->can_channel &&
                previous->can_tx_id == config->can_tx_id) {
                return ROBOT_STATUS_INVALID_ARGUMENT;
            }
        }
        DmContext *context = &s_contexts[config->motor_id];
        context->configured = true;
        context->enable_frames_left = DM_ENABLE_WARMUP_FRAMES;
        VendorControl_Init(&context->control, config);
    }
    return ROBOT_STATUS_OK;
}

static RobotStatus dm_command_current(uint8_t motor_id, int16_t current)
{
    if (motor_id >= VENDOR_MOTOR_CAPACITY || !s_contexts[motor_id].configured) {
        return ROBOT_STATUS_NOT_READY;
    }
    DmContext *context = &s_contexts[motor_id];
    if (!context->enabled) context->enable_frames_left = DM_ENABLE_WARMUP_FRAMES;
    context->enabled = true;
    context->command = current;
    context->pending = true;
    return ROBOT_STATUS_OK;
}

static RobotStatus dm_stop(uint8_t motor_id)
{
    if (motor_id >= VENDOR_MOTOR_CAPACITY || !s_contexts[motor_id].configured) {
        return ROBOT_STATUS_NOT_READY;
    }
    s_contexts[motor_id].enabled = false;
    s_contexts[motor_id].command = 0;
    s_contexts[motor_id].pending = true;
    return ROBOT_STATUS_OK;
}

static RobotStatus dm_compute_current(uint8_t motor_id, MotorControlMode_e mode,
                                      float setpoint, int16_t application_current,
                                      int16_t *output)
{
    if (motor_id >= VENDOR_MOTOR_CAPACITY || !s_contexts[motor_id].configured) {
        return ROBOT_STATUS_NOT_READY;
    }
    return VendorControl_Compute(&s_contexts[motor_id].control, mode, setpoint,
                                 application_current, 32767, output);
}

static RobotStatus dm_snapshot(uint8_t motor_id, MotorSnapshot *snapshot)
{
    if (motor_id >= VENDOR_MOTOR_CAPACITY || !snapshot ||
        !s_contexts[motor_id].configured) return ROBOT_STATUS_INVALID_ARGUMENT;
    *snapshot = s_contexts[motor_id].control.snapshot;
    return snapshot->feedback_valid ? ROBOT_STATUS_OK : ROBOT_STATUS_NOT_READY;
}

static void dm_on_can_frame(const CanRxFrame *frame)
{
    if (!frame || frame->dlc != 8U) return;
    for (uint8_t motor_id = 0U; motor_id < VENDOR_MOTOR_CAPACITY; ++motor_id) {
        DmContext *context = &s_contexts[motor_id];
        const MotorConfig_t *config = context->control.config;
        if (!context->configured || !config || frame->channel != config->can_channel ||
            frame->std_id != config->can_rx_id) continue;
        DmMotorFeedback feedback;
        const DmMotorLimits limits = protocol_limits(config);
        if (!DmMotorProtocol_DecodeFeedback(&limits, frame->data, &feedback)) return;
        context->control.snapshot.feedback_valid = true;
        context->control.snapshot.position = feedback.position;
        context->control.snapshot.speed = feedback.velocity;
        context->control.snapshot.torque_or_current = feedback.torque;
        context->control.snapshot.feedback_timestamp_ms = frame->tick_ms;
        return;
    }
}

static void dm_reset_control(uint8_t motor_id)
{
    if (motor_id < VENDOR_MOTOR_CAPACITY && s_contexts[motor_id].configured) {
        VendorControl_Reset(&s_contexts[motor_id].control);
    }
}

static void dm_flush(void)
{
    for (uint8_t motor_id = 0U; motor_id < VENDOR_MOTOR_CAPACITY; ++motor_id) {
        DmContext *context = &s_contexts[motor_id];
        const MotorConfig_t *config = context->control.config;
        if (!context->configured || !context->pending || !config) continue;
        uint8_t data[8];
        if (!context->enabled) {
            DmMotorProtocol_BuildStateCommand(DM_DISABLE_COMMAND, data);
        } else if (context->enable_frames_left > 0U) {
            DmMotorProtocol_BuildStateCommand(DM_ENABLE_COMMAND, data);
            --context->enable_frames_left;
        } else {
            const DmMotorLimits limits = protocol_limits(config);
            const float magnitude = fmaxf(fabsf(limits.torque_min),
                                          fabsf(limits.torque_max));
            const float torque = (float)context->command * magnitude / 32767.0f;
            if (!DmMotorProtocol_EncodeMit(&limits, 0.0f, 0.0f, 0.0f, 0.0f,
                                           torque, data)) continue;
        }
        if (BspCan_Write(bsp_channel(config->can_channel), config->can_tx_id,
                         data, sizeof(data))) {
            context->pending = context->enabled && context->enable_frames_left > 0U;
        }
    }
}

const MotorAdapterOps *DmMotorAdapter_Get(void)
{
    static const MotorAdapterOps operations = {
        .vendor = MOTOR_VENDOR_DM,
        .name = "DM MIT adapter",
        .implemented = true,
        .init = dm_init,
        .validate = dm_validate,
        .command_current = dm_command_current,
        .stop = dm_stop,
        .compute_current = dm_compute_current,
        .snapshot = dm_snapshot,
        .on_can_frame = dm_on_can_frame,
        .reset_control = dm_reset_control,
        .flush = dm_flush
    };
    return &operations;
}
