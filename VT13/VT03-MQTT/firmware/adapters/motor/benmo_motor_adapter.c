/* 本末 BM1505B 适配器：配置帧和两组四电机命令统一在刷新阶段发送。 */
#include "motor_adapter.h"
#include "vendor_control.h"
#include "benmo_motor_protocol.h"
#include "bsp_can.h"
#include <stddef.h>
#include <string.h>

#define VENDOR_MOTOR_CAPACITY 16U
#define BENMO_STARTUP_DISABLE_CYCLES 10U
#define BENMO_STARTUP_ENABLE_CYCLES 10U
#define BENMO_STARTUP_CONFIG_CYCLES 10U
#define BENMO_MODE_DISABLED 0x09U
#define BENMO_MODE_ENABLED  0x0AU

typedef struct {
    bool configured;
    bool pending;
    int16_t command;
    VendorControl control;
} BenmoContext;

static BenmoContext s_contexts[VENDOR_MOTOR_CAPACITY];
static uint8_t s_startup_cycle[CAN_CHANNEL_COUNT];

static BspCanChannel bsp_channel(CAN_Channel_t channel)
{
    return channel == CAN_CHANNEL_2 ? BSP_CAN_CHANNEL_2 : BSP_CAN_CHANNEL_1;
}

static RobotStatus benmo_validate(const MotorConfig_t *config)
{
    if (!config || config->vendor != MOTOR_VENDOR_BENMO ||
        config->type != MOTOR_TYPE_VENDOR_DEFINED ||
        config->motor_id >= VENDOR_MOTOR_CAPACITY ||
        (config->direction != 1 && config->direction != -1) ||
        config->can_channel >= CAN_CHANNEL_COUNT ||
        config->protocol.benmo.motor_address < 1U ||
        config->protocol.benmo.motor_address > 8U ||
        (config->protocol.benmo.command_mode != BENMO_CONTROL_RAW_CURRENT &&
         config->protocol.benmo.command_mode != BENMO_CONTROL_SPEED_RPM_X10) ||
        config->protocol.benmo.command_limit <= 0 ||
        !(config->protocol.benmo.feedback_mode == 0x00U ||
          config->protocol.benmo.feedback_mode == 0x80U ||
          config->protocol.benmo.feedback_mode == 0x01U ||
          config->protocol.benmo.feedback_mode == 0x0AU ||
          config->protocol.benmo.feedback_mode == 0x32U ||
          config->protocol.benmo.feedback_mode == 0x40U ||
          config->protocol.benmo.feedback_mode == 0x7FU) ||
        config->tx_slot != (uint8_t)((config->protocol.benmo.motor_address - 1U) % 4U) ||
        config->can_tx_id != (config->protocol.benmo.motor_address <= 4U
                                  ? BENMO_CONTROL_GROUP_1_ID
                                  : BENMO_CONTROL_GROUP_2_ID)) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    return ROBOT_STATUS_OK;
}

static RobotStatus benmo_init(const RobotConfig_t *robot)
{
    if (!robot || (robot->total_motor_count > 0U && !robot->motor_configs)) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    memset(s_contexts, 0, sizeof(s_contexts));
    memset(s_startup_cycle, 0, sizeof(s_startup_cycle));
    bool used_addresses[CAN_CHANNEL_COUNT][8] = {{false}};
    for (uint8_t index = 0U; index < robot->total_motor_count; ++index) {
        const MotorConfig_t *config = &robot->motor_configs[index];
        if (config->vendor != MOTOR_VENDOR_BENMO) continue;
        RobotStatus status = benmo_validate(config);
        const uint8_t address_index =
            (uint8_t)(config->protocol.benmo.motor_address - 1U);
        if (status != ROBOT_STATUS_OK ||
            used_addresses[config->can_channel][address_index]) {
            return ROBOT_STATUS_INVALID_ARGUMENT;
        }
        used_addresses[config->can_channel][address_index] = true;
        BenmoContext *context = &s_contexts[config->motor_id];
        context->configured = true;
        VendorControl_Init(&context->control, config);
    }
    return ROBOT_STATUS_OK;
}

static RobotStatus benmo_command_current(uint8_t motor_id, int16_t current)
{
    if (motor_id >= VENDOR_MOTOR_CAPACITY || !s_contexts[motor_id].configured) {
        return ROBOT_STATUS_NOT_READY;
    }
    BenmoContext *context = &s_contexts[motor_id];
    const int16_t limit = context->control.config->protocol.benmo.command_limit;
    if (current > limit) current = limit;
    if (current < -limit) current = (int16_t)-limit;
    context->command = current;
    context->pending = true;
    return ROBOT_STATUS_OK;
}

static RobotStatus benmo_stop(uint8_t motor_id)
{
    return benmo_command_current(motor_id, 0);
}

static RobotStatus benmo_compute_current(uint8_t motor_id, MotorControlMode_e mode,
                                         float setpoint, int16_t application_current,
                                         int16_t *output)
{
    if (motor_id >= VENDOR_MOTOR_CAPACITY || !s_contexts[motor_id].configured) {
        return ROBOT_STATUS_NOT_READY;
    }
    BenmoContext *context = &s_contexts[motor_id];
    return VendorControl_Compute(&context->control, mode, setpoint,
                                 application_current,
                                 context->control.config->protocol.benmo.command_limit,
                                 output);
}

static RobotStatus benmo_snapshot(uint8_t motor_id, MotorSnapshot *snapshot)
{
    if (motor_id >= VENDOR_MOTOR_CAPACITY || !snapshot ||
        !s_contexts[motor_id].configured) return ROBOT_STATUS_INVALID_ARGUMENT;
    *snapshot = s_contexts[motor_id].control.snapshot;
    return snapshot->feedback_valid ? ROBOT_STATUS_OK : ROBOT_STATUS_NOT_READY;
}

static void benmo_on_can_frame(const CanRxFrame *frame)
{
    if (!frame || frame->dlc != 8U) return;
    for (uint8_t motor_id = 0U; motor_id < VENDOR_MOTOR_CAPACITY; ++motor_id) {
        BenmoContext *context = &s_contexts[motor_id];
        const MotorConfig_t *config = context->control.config;
        if (!context->configured || !config || frame->channel != config->can_channel ||
            frame->std_id != config->can_rx_id) continue;
        BenmoMotorFeedback feedback;
        if (!BenmoMotorProtocol_DecodeFeedback(frame->data, &feedback)) return;
        context->control.snapshot.feedback_valid = true;
        context->control.snapshot.position = (float)feedback.angle_raw;
        context->control.snapshot.speed = (float)feedback.velocity_raw;
        context->control.snapshot.torque_or_current = (float)feedback.current_raw;
        context->control.snapshot.feedback_timestamp_ms = frame->tick_ms;
        return;
    }
}

static void benmo_reset_control(uint8_t motor_id)
{
    if (motor_id < VENDOR_MOTOR_CAPACITY && s_contexts[motor_id].configured) {
        VendorControl_Reset(&s_contexts[motor_id].control);
    }
}

static bool build_config_frame(CAN_Channel_t channel, uint8_t value,
                               bool feedback_mode, uint8_t data[8])
{
    memset(data, 0, 8U);
    bool has_motor = false;
    for (uint8_t motor_id = 0U; motor_id < VENDOR_MOTOR_CAPACITY; ++motor_id) {
        const BenmoContext *context = &s_contexts[motor_id];
        const MotorConfig_t *config = context->control.config;
        if (!context->configured || !config || config->can_channel != channel) continue;
        const uint8_t slot_value = feedback_mode
                                       ? config->protocol.benmo.feedback_mode
                                       : value;
        has_motor = BenmoMotorProtocol_SetSlot(
                        config->protocol.benmo.motor_address, slot_value, data) || has_motor;
    }
    return has_motor;
}

static bool send_startup_frame(CAN_Channel_t channel)
{
    uint8_t data[8];
    const uint8_t cycle = s_startup_cycle[channel];
    if (cycle < BENMO_STARTUP_DISABLE_CYCLES) {
        if (!build_config_frame(channel, BENMO_MODE_DISABLED, false, data)) return false;
        return BspCan_Write(bsp_channel(channel), BENMO_MODE_FRAME_ID, data, sizeof(data));
    }
    if (cycle < BENMO_STARTUP_DISABLE_CYCLES + BENMO_STARTUP_ENABLE_CYCLES) {
        if (!build_config_frame(channel, BENMO_MODE_ENABLED, false, data)) return false;
        return BspCan_Write(bsp_channel(channel), BENMO_MODE_FRAME_ID, data, sizeof(data));
    }
    if (cycle < BENMO_STARTUP_DISABLE_CYCLES + BENMO_STARTUP_ENABLE_CYCLES +
                BENMO_STARTUP_CONFIG_CYCLES) {
        memset(data, 0, sizeof(data));
        bool has_motor = false;
        for (uint8_t motor_id = 0U; motor_id < VENDOR_MOTOR_CAPACITY; ++motor_id) {
            const BenmoContext *context = &s_contexts[motor_id];
            const MotorConfig_t *config = context->control.config;
            if (!context->configured || !config || config->can_channel != channel) continue;
            has_motor = BenmoMotorProtocol_SetSlot(
                            config->protocol.benmo.motor_address,
                            (uint8_t)config->protocol.benmo.command_mode, data) || has_motor;
        }
        if (!has_motor || !BspCan_Write(bsp_channel(channel), BENMO_MODE_FRAME_ID,
                                        data, sizeof(data))) return false;
        if (!build_config_frame(channel, 0U, true, data)) return false;
        return BspCan_Write(bsp_channel(channel), BENMO_FEEDBACK_MODE_FRAME_ID,
                            data, sizeof(data));
    }
    return true;
}

static void send_command_groups(CAN_Channel_t channel)
{
    for (uint8_t group = 0U; group < 2U; ++group) {
        int16_t commands[4] = {0};
        bool has_motor = false;
        bool has_pending = false;
        for (uint8_t motor_id = 0U; motor_id < VENDOR_MOTOR_CAPACITY; ++motor_id) {
            BenmoContext *context = &s_contexts[motor_id];
            const MotorConfig_t *config = context->control.config;
            if (!context->configured || !config || config->can_channel != channel ||
                (uint8_t)((config->protocol.benmo.motor_address - 1U) / 4U) != group) continue;
            has_motor = true;
            has_pending = has_pending || context->pending;
            commands[config->tx_slot] = context->command;
        }
        if (!has_motor || !has_pending) continue;
        uint8_t data[8];
        const uint16_t identifier = group == 0U ? BENMO_CONTROL_GROUP_1_ID
                                                : BENMO_CONTROL_GROUP_2_ID;
        if (!BenmoMotorProtocol_BuildGroup(commands, data) ||
            !BspCan_Write(bsp_channel(channel), identifier, data, sizeof(data))) continue;
        for (uint8_t motor_id = 0U; motor_id < VENDOR_MOTOR_CAPACITY; ++motor_id) {
            BenmoContext *context = &s_contexts[motor_id];
            const MotorConfig_t *config = context->control.config;
            if (context->configured && config && config->can_channel == channel &&
                (uint8_t)((config->protocol.benmo.motor_address - 1U) / 4U) == group) {
                context->pending = false;
            }
        }
    }
}

static void benmo_flush(void)
{
    for (uint8_t channel = 0U; channel < CAN_CHANNEL_COUNT; ++channel) {
        const uint8_t startup_total = BENMO_STARTUP_DISABLE_CYCLES +
                                      BENMO_STARTUP_ENABLE_CYCLES +
                                      BENMO_STARTUP_CONFIG_CYCLES;
        if (s_startup_cycle[channel] < startup_total) {
            if (send_startup_frame((CAN_Channel_t)channel)) ++s_startup_cycle[channel];
            continue;
        }
        send_command_groups((CAN_Channel_t)channel);
    }
}

const MotorAdapterOps *BenmoMotorAdapter_Get(void)
{
    static const MotorAdapterOps operations = {
        .vendor = MOTOR_VENDOR_BENMO,
        .name = "BenMo BM1505B adapter",
        .implemented = true,
        .init = benmo_init,
        .validate = benmo_validate,
        .command_current = benmo_command_current,
        .stop = benmo_stop,
        .compute_current = benmo_compute_current,
        .snapshot = benmo_snapshot,
        .on_can_frame = benmo_on_can_frame,
        .reset_control = benmo_reset_control,
        .flush = benmo_flush
    };
    return &operations;
}
