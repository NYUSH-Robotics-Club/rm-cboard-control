/*
 * 统一校验电机配置、选择厂商和控制模式，并集中刷新发送数据。
 * 不匹配的模式会安全停机并返回明确错误。
 */
#include "motor_service.h"
#include "message_center.h"
#include "robot_config.h"
#include <stddef.h>

static bool s_mode_override_valid[16];
static MotorControlMode_e s_mode_override[16];
static bool s_initialized;

MotorCommandUnit MotorService_GetCommandUnit(uint8_t motor_id)
{
    const MotorConfig_t *config = MotorService_GetConfig(motor_id);
    if (!config || config->vendor != MOTOR_VENDOR_DJI)
        return MOTOR_COMMAND_UNIT_UNKNOWN;
    if (config->type == MOTOR_TYPE_M3508 || config->type == MOTOR_TYPE_M2006)
        return MOTOR_COMMAND_UNIT_CURRENT_COUNTS;
    if (config->type != MOTOR_TYPE_GM6020) return MOTOR_COMMAND_UNIT_UNKNOWN;
    switch (config->protocol.dji.gm6020_mode) {
        case GM6020_COMMAND_CURRENT: return MOTOR_COMMAND_UNIT_CURRENT_COUNTS;
        case GM6020_COMMAND_VOLTAGE: return MOTOR_COMMAND_UNIT_VOLTAGE_COUNTS;
        default: return MOTOR_COMMAND_UNIT_UNKNOWN;
    }
}

enum { MOTOR_ADAPTER_COUNT = 4U };

static const MotorAdapterOps *adapter_at(size_t index)
{
    switch (index) {
        case 0U: return DjiMotorAdapter_Get();
        case 1U: return DmMotorAdapter_Get();
        case 2U: return BenmoMotorAdapter_Get();
        case 3U: return LkMotorAdapter_Get();
        default: return NULL;
    }
}

static void flush_after_dispatch(void *user_data)
{
    (void)user_data;
    MotorService_Flush();
}

static void on_can_frame(const MsgEvent *event, void *user_data)
{
    (void)user_data;
    if (!event || event->size != sizeof(CanRxFrame)) return;
    const CanRxFrame *frame = (const CanRxFrame *)event->data;
    if (!frame->is_standard_frame || !frame->is_data_frame || frame->dlc != 8U) {
        return;
    }
    for (size_t index = 0U; index < MOTOR_ADAPTER_COUNT; ++index) {
        const MotorAdapterOps *adapter = adapter_at(index);
        if (adapter && adapter->implemented && adapter->on_can_frame) {
            adapter->on_can_frame(frame);
        }
    }
}

static const MotorConfig_t *find_config(uint8_t motor_id)
{
    const RobotConfig_t *robot = RobotConfig_Get();
    if (!robot || !robot->motor_configs) {
        return NULL;
    }
    for (uint8_t i = 0; i < robot->total_motor_count; ++i) {
        if (robot->motor_configs[i].motor_id == motor_id) {
            return &robot->motor_configs[i];
        }
    }
    return NULL;
}

static MotorControlMode_e effective_mode(const MotorConfig_t *config)
{
    if (!config) {
        return MOTOR_CONTROL_DISABLED;
    }
    if (config->motor_id < 16U && s_mode_override_valid[config->motor_id]) {
        return s_mode_override[config->motor_id];
    }
    return config->control_mode;
}

static RobotStatus validate_robot_motors(const RobotConfig_t *robot)
{
    if (!robot || (robot->total_motor_count > 0U && !robot->motor_configs) ||
        robot->total_motor_count > 16U) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }

    bool used_logical_ids[16] = {false};
    for (uint8_t index = 0U; index < robot->total_motor_count; ++index) {
        const MotorConfig_t *config = &robot->motor_configs[index];
        if (config->motor_id >= 16U || used_logical_ids[config->motor_id] ||
            config->can_channel >= CAN_CHANNEL_COUNT ||
            (config->direction != 1 && config->direction != -1)) {
            return ROBOT_STATUS_INVALID_ARGUMENT;
        }
        used_logical_ids[config->motor_id] = true;

        const MotorAdapterOps *adapter = MotorAdapter_Get(config->vendor);
        if (!adapter || !adapter->implemented || !adapter->validate ||
            adapter->validate(config) != ROBOT_STATUS_OK) {
            return ROBOT_STATUS_UNSUPPORTED;
        }

        for (uint8_t other = 0U; other < index; ++other) {
            const MotorConfig_t *previous = &robot->motor_configs[other];
            if (previous->can_channel == config->can_channel &&
                previous->can_rx_id == config->can_rx_id) {
                return ROBOT_STATUS_INVALID_ARGUMENT;
            }
        }
    }
    return ROBOT_STATUS_OK;
}

RobotStatus MotorService_Init(void)
{
    if (s_initialized) return ROBOT_STATUS_OK;

    const RobotConfig_t *robot = RobotConfig_Get();
    if (!robot) return ROBOT_STATUS_NOT_READY;
    RobotStatus validation = validate_robot_motors(robot);
    if (validation != ROBOT_STATUS_OK) return validation;
    for (size_t index = 0U; index < MOTOR_ADAPTER_COUNT; ++index) {
        const MotorAdapterOps *adapter = adapter_at(index);
        if (adapter && adapter->implemented && adapter->init) {
            RobotStatus status = adapter->init(robot);
            if (status != ROBOT_STATUS_OK) return status;
        }
    }
    if (MsgCenter_Subscribe(TOPIC_CAN_RX, on_can_frame, NULL) != 0 ||
        MsgCenter_AddAfterDispatchHook(flush_after_dispatch, NULL) != 0) {
        return ROBOT_STATUS_CAPACITY_EXCEEDED;
    }
    s_initialized = true;
    return ROBOT_STATUS_OK;
}

const MotorConfig_t *MotorService_GetConfig(uint8_t motor_id)
{
    return find_config(motor_id);
}

uint8_t MotorService_FindByRole(MotorRole_e role,
                                uint8_t *motor_ids,
                                uint8_t max_count)
{
    const RobotConfig_t *robot = RobotConfig_Get();
    if (!robot || !motor_ids || max_count == 0U) {
        return 0U;
    }

    uint8_t found = 0U;
    for (uint8_t i = 0; i < robot->total_motor_count && found < max_count; ++i) {
        if (robot->motor_configs[i].role == role) {
            motor_ids[found++] = robot->motor_configs[i].motor_id;
        }
    }
    return found;
}

RobotStatus MotorService_GetSnapshot(uint8_t motor_id, MotorSnapshot *snapshot)
{
    const MotorConfig_t *config = find_config(motor_id);
    if (!config || !snapshot) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    const MotorAdapterOps *adapter = MotorAdapter_Get(config->vendor);
    if (!adapter || !adapter->implemented || !adapter->snapshot) {
        return ROBOT_STATUS_UNSUPPORTED;
    }
    return adapter->snapshot(motor_id, snapshot);
}

RobotStatus MotorService_CommandCurrent(uint8_t motor_id, int16_t current)
{
    const MotorConfig_t *config = find_config(motor_id);
    if (!config) {
        return ROBOT_STATUS_NOT_FOUND;
    }
    RobotStatus init_status = MotorService_Init();
    if (init_status != ROBOT_STATUS_OK) {
        return init_status;
    }

    const MotorAdapterOps *adapter = MotorAdapter_Get(config->vendor);
    if (!adapter || !adapter->implemented || !adapter->command_current) {
        return ROBOT_STATUS_UNSUPPORTED;
    }
    if (adapter->validate && adapter->validate(config) != ROBOT_STATUS_OK) {
        return ROBOT_STATUS_UNSUPPORTED;
    }

    MotorControlMode_e mode = effective_mode(config);
    if (mode == MOTOR_CONTROL_DISABLED) {
        if (adapter->stop) {
            (void)adapter->stop(motor_id);
        } else {
            (void)adapter->command_current(motor_id, 0);
        }
        return ROBOT_STATUS_NOT_READY;
    }
    if (mode != MOTOR_CONTROL_APPLICATION &&
        mode != MOTOR_CONTROL_OPEN_LOOP_CURRENT) {
        if (adapter->stop) {
            (void)adapter->stop(motor_id);
        } else {
            (void)adapter->command_current(motor_id, 0);
        }
        return ROBOT_STATUS_MODE_MISMATCH;
    }
    return adapter->command_current(motor_id, current);
}

RobotStatus MotorService_CommandConfigured(uint8_t motor_id,
                                           float setpoint,
                                           int16_t application_current)
{
    const MotorConfig_t *config = find_config(motor_id);
    if (!config) {
        return ROBOT_STATUS_NOT_FOUND;
    }
    RobotStatus init_status = MotorService_Init();
    if (init_status != ROBOT_STATUS_OK) {
        return init_status;
    }

    const MotorAdapterOps *adapter = MotorAdapter_Get(config->vendor);
    if (!adapter || !adapter->implemented || !adapter->compute_current ||
        !adapter->command_current) {
        return ROBOT_STATUS_UNSUPPORTED;
    }
    if (adapter->validate && adapter->validate(config) != ROBOT_STATUS_OK) {
        return ROBOT_STATUS_UNSUPPORTED;
    }

    int16_t output = 0;
    MotorControlMode_e mode = effective_mode(config);
    RobotStatus status = adapter->compute_current(motor_id,
                                                  mode,
                                                  setpoint,
                                                  application_current,
                                                  &output);
    if (status != ROBOT_STATUS_OK &&
        mode != MOTOR_CONTROL_DISABLED) {
        (void)adapter->command_current(motor_id, 0);
        return status;
    }
    if (mode == MOTOR_CONTROL_DISABLED && adapter->stop) {
        (void)adapter->stop(motor_id);
        return status;
    }
    (void)adapter->command_current(motor_id, output);
    return status;
}

RobotStatus MotorService_SetControlMode(uint8_t motor_id,
                                        MotorControlMode_e mode)
{
    const MotorConfig_t *config = find_config(motor_id);
    if (!config || motor_id >= 16U) {
        return ROBOT_STATUS_NOT_FOUND;
    }
    if (mode < MOTOR_CONTROL_APPLICATION ||
        mode > MOTOR_CONTROL_POSITION_SPEED_CASCADE) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    s_mode_override[motor_id] = mode;
    s_mode_override_valid[motor_id] = true;
    const MotorAdapterOps *adapter = MotorAdapter_Get(config->vendor);
    if (adapter && adapter->reset_control) {
        adapter->reset_control(motor_id);
    }
    return ROBOT_STATUS_OK;
}

RobotStatus MotorService_ClearControlModeOverride(uint8_t motor_id)
{
    const MotorConfig_t *config = find_config(motor_id);
    if (!config || motor_id >= 16U) {
        return ROBOT_STATUS_NOT_FOUND;
    }
    s_mode_override_valid[motor_id] = false;
    const MotorAdapterOps *adapter = MotorAdapter_Get(config->vendor);
    if (adapter && adapter->reset_control) {
        adapter->reset_control(motor_id);
    }
    return ROBOT_STATUS_OK;
}

MotorControlMode_e MotorService_GetControlMode(uint8_t motor_id)
{
    return effective_mode(find_config(motor_id));
}

bool MotorService_IsVendorImplemented(MotorVendor_e vendor)
{
    const MotorAdapterOps *adapter = MotorAdapter_Get(vendor);
    return adapter && adapter->implemented;
}

const char *MotorService_VendorName(MotorVendor_e vendor)
{
    const MotorAdapterOps *adapter = MotorAdapter_Get(vendor);
    return adapter ? adapter->name : "unknown";
}

void MotorService_Flush(void)
{
    for (size_t index = 0U; index < MOTOR_ADAPTER_COUNT; ++index) {
        const MotorAdapterOps *adapter = adapter_at(index);
        if (adapter && adapter->implemented && adapter->flush) {
            adapter->flush();
        }
    }
}
