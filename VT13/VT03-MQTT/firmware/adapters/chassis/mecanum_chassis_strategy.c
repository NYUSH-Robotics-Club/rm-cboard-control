/*
 * 把车体移动指令换算成四个麦轮的目标转速。
 * 本文件只做数学换算，不读取电机，也不发送 CAN 数据。
 */
#include "chassis_strategy.h"
#include <string.h>

static RobotStatus mecanum_compute(const ChassisKinematicsInput *input,
                                   ChassisKinematicsOutput *output)
{
    if (!input || !output || input->max_drive_speed < 0.0f) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }
    memset(output, 0, sizeof(*output));

    /* Preserve the established project coordinate convention and scaling. */
    const float scale = input->max_drive_speed * 0.5f;
    const float vx = input->vx * scale;
    const float vy = input->vy * scale;
    const float omega = input->wz * scale;

    output->drive_speed[0] = vx - vy + omega;
    output->drive_speed[1] = vx + vy - omega;
    output->drive_speed[2] = vx - vy - omega;
    output->drive_speed[3] = vx + vy + omega;
    output->drive_count = 4U;
    return ROBOT_STATUS_OK;
}

const ChassisStrategy *MecanumChassisStrategy_Get(void)
{
    static const ChassisStrategy strategy = {
        .type = CHASSIS_TYPE_MECANUM,
        .name = "mecanum-4",
        .execution = CHASSIS_EXECUTION_KINEMATICS,
        .implemented = true,
        .compute = mecanum_compute
    };
    return &strategy;
}
