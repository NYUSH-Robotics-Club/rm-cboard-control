/* 使用车型配置中的两组 PID；协议适配器只负责反馈单位和 CAN 帧。 */
#include "vendor_control.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static int16_t clamp_command(float value, int16_t limit)
{
    if (value > (float)limit) value = (float)limit;
    if (value < (float)-limit) value = (float)-limit;
    return (int16_t)value;
}

void VendorControl_Init(VendorControl *control, const MotorConfig_t *config)
{
    if (!control || !config) return;
    memset(control, 0, sizeof(*control));
    control->config = config;
    control->snapshot.motor_id = config->motor_id;
    control->snapshot.initialized = true;
    PID_Init(&control->outer, config->pid_outer.kp, config->pid_outer.ki,
             config->pid_outer.kd, config->pid_outer.output_max,
             config->pid_outer.integral_max);
    PID_Init(&control->inner, config->pid_inner.kp, config->pid_inner.ki,
             config->pid_inner.kd, config->pid_inner.output_max,
             config->pid_inner.integral_max);
}

RobotStatus VendorControl_Compute(VendorControl *control,
                                  MotorControlMode_e mode,
                                  float setpoint,
                                  int16_t application_current,
                                  int16_t command_limit,
                                  int16_t *output)
{
    if (!control || !control->config || !output || command_limit <= 0 ||
        !isfinite(setpoint)) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }

    float command = 0.0f;
    switch (mode) {
        case MOTOR_CONTROL_APPLICATION:
            command = (float)application_current;
            break;
        case MOTOR_CONTROL_DISABLED:
            *output = 0;
            return ROBOT_STATUS_NOT_READY;
        case MOTOR_CONTROL_OPEN_LOOP_CURRENT:
            command = setpoint;
            break;
        case MOTOR_CONTROL_SPEED:
            if (!control->snapshot.feedback_valid) return ROBOT_STATUS_NOT_READY;
            command = PID_Calculate(&control->outer, setpoint,
                                    control->snapshot.speed);
            break;
        case MOTOR_CONTROL_POSITION:
            if (!control->snapshot.feedback_valid) return ROBOT_STATUS_NOT_READY;
            command = PID_Calculate(&control->outer, setpoint,
                                    control->snapshot.position);
            break;
        case MOTOR_CONTROL_POSITION_SPEED_CASCADE:
            if (!control->snapshot.feedback_valid) return ROBOT_STATUS_NOT_READY;
            command = PID_Calculate(&control->outer, setpoint,
                                    control->snapshot.position);
            command = PID_Calculate(&control->inner, command,
                                    control->snapshot.speed);
            break;
        default:
            return ROBOT_STATUS_MODE_MISMATCH;
    }

    command *= (float)control->config->direction;
    *output = clamp_command(command, command_limit);
    return ROBOT_STATUS_OK;
}

void VendorControl_Reset(VendorControl *control)
{
    if (!control || !control->config) return;
    PID_Reset(&control->outer);
    PID_Reset(&control->inner);
}
