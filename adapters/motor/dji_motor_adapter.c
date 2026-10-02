/*
 * 把统一电机命令交给现有 DJI 电机驱动。
 * 这里负责限幅和控制模式选择，应用层不需要拼接 CAN 帧。
 */
#include "motor_adapter.h"
#include "motor_driver.h"
#include "bsp_can.h"
#include "dji_motor_protocol.h"
#include <stddef.h>

static RobotStatus dji_validate(const MotorConfig_t *config)
{
    if (!config || config->vendor != MOTOR_VENDOR_DJI) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    switch (config->type) {
        case MOTOR_TYPE_M3508:
        case MOTOR_TYPE_GM6020:
        case MOTOR_TYPE_M2006:
            return DjiMotor_CommandLimit(config) > 0
                       ? ROBOT_STATUS_OK : ROBOT_STATUS_INVALID_ARGUMENT;
        default:
            return ROBOT_STATUS_UNSUPPORTED;
    }
}

static RobotStatus dji_command_current(uint8_t motor_id, int16_t current)
{
    MotorContext_t *context = MotorDriver_GetContext(motor_id);
    if (!context || !context->initialized) {
        return ROBOT_STATUS_NOT_READY;
    }
    const int16_t limit = DjiMotor_CommandLimit(context->config);
    if (limit == 0) return ROBOT_STATUS_INVALID_ARGUMENT;
    if (current > limit) current = limit;
    if (current < -limit) current = (int16_t)-limit;
    MotorDriver_SendCurrent(motor_id, current);
    return ROBOT_STATUS_OK;
}

static RobotStatus dji_stop(uint8_t motor_id)
{
    return dji_command_current(motor_id, 0);
}

static RobotStatus dji_compute_current(uint8_t motor_id,
                                       MotorControlMode_e mode,
                                       float setpoint,
                                       int16_t application_current,
                                       int16_t *output_current)
{
    if (!output_current) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    MotorContext_t *context = MotorDriver_GetContext(motor_id);
    if (!context || !context->initialized || !context->config) {
        *output_current = 0;
        return ROBOT_STATUS_NOT_READY;
    }
    switch (mode) {
        case MOTOR_CONTROL_APPLICATION:
            *output_current = application_current;
            return ROBOT_STATUS_OK;
        case MOTOR_CONTROL_DISABLED:
            *output_current = 0;
            return ROBOT_STATUS_NOT_READY;
        case MOTOR_CONTROL_OPEN_LOOP_CURRENT:
        {
            const float limit = (float)DjiMotor_CommandLimit(context->config);
            if (setpoint > limit) setpoint = limit;
            if (setpoint < -limit) setpoint = -limit;
            *output_current = (int16_t)setpoint;
            return ROBOT_STATUS_OK;
        }
        case MOTOR_CONTROL_SPEED:
            *output_current = MotorDriver_ComputeCurrent(motor_id, setpoint, false);
            return ROBOT_STATUS_OK;
        case MOTOR_CONTROL_POSITION:
        {
            if (!context->angle_initialized) {
                *output_current = 0;
                return ROBOT_STATUS_NOT_READY;
            }
            float target = setpoint;
            if (context->type == MOTOR_TYPE_GM6020) {
                if (target < context->config->limits.gm6020.angle_min) {
                    target = context->config->limits.gm6020.angle_min;
                }
                if (target > context->config->limits.gm6020.angle_max) {
                    target = context->config->limits.gm6020.angle_max;
                }
            }
            context->angle_target = target;
            float current = PID_Calculate(&context->pid_outer,
                                          target,
                                          (float)context->angle_raw);
            if (context->type == MOTOR_TYPE_GM6020 &&
                context->role == MOTOR_ROLE_GIMBAL_PITCH) {
                current += context->config->limits.gm6020.gravity_compensation;
            }
            current *= (float)context->config->direction;
            const float limit = (float)DjiMotor_CommandLimit(context->config);
            if (current > limit) current = limit;
            if (current < -limit) current = -limit;
            *output_current = (int16_t)current;
            return ROBOT_STATUS_OK;
        }
        case MOTOR_CONTROL_POSITION_SPEED_CASCADE:
            *output_current = MotorDriver_ComputeCurrent(motor_id, setpoint, true);
            return ROBOT_STATUS_OK;
        default:
            *output_current = 0;
            return ROBOT_STATUS_MODE_MISMATCH;
    }
}

static RobotStatus dji_snapshot(uint8_t motor_id, MotorSnapshot *snapshot)
{
    MotorContext_t *context = MotorDriver_GetContext(motor_id);
    if (!context || !snapshot) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    snapshot->motor_id = motor_id;
    snapshot->initialized = context->initialized;
    snapshot->feedback_valid = context->angle_initialized;
    snapshot->position = (float)context->angle_raw;
    snapshot->speed = (float)context->speed_rpm;
    snapshot->torque_or_current = (float)context->feedback_current;
    snapshot->feedback_timestamp_ms = context->last_feedback_time;
    return context->initialized ? ROBOT_STATUS_OK : ROBOT_STATUS_NOT_READY;
}

static void dji_flush(void)
{
    MotorDriver_FlushAll();
}

static void dji_reset_control(uint8_t motor_id)
{
    MotorDriver_ResetPID(motor_id);
}

const MotorAdapterOps *DjiMotorAdapter_Get(void)
{
    static const MotorAdapterOps ops = {
        .vendor = MOTOR_VENDOR_DJI,
        .name = "DJI legacy CAN adapter",
        .implemented = true,
        .init = 0,
        .validate = dji_validate,
        .command_current = dji_command_current,
        .stop = dji_stop,
        .compute_current = dji_compute_current,
        .snapshot = dji_snapshot,
        .on_can_frame = 0,
        .reset_control = dji_reset_control,
        .flush = dji_flush,
        .read_trace = BspCan_TraceRead
    };
    return &ops;
}
