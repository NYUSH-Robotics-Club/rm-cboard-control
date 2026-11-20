/*
 * 说明当前舵轮由旧哨兵控制器执行。
 * 通用舵轮几何尚未提供，因此这里不重复计算运动学。
 */
#include "chassis_strategy.h"

static RobotStatus legacy_swerve_compute(const ChassisKinematicsInput *input,
                                         ChassisKinematicsOutput *output)
{
    (void)input;
    (void)output;
    /* Current two-steer sentry behavior remains in sentry_controller.c. */
    return ROBOT_STATUS_UNSUPPORTED;
}

const ChassisStrategy *SwerveChassisStrategy_Get(void)
{
    static const ChassisStrategy strategy = {
        .type = CHASSIS_TYPE_SWERVE,
        .name = "legacy-sentry-swerve",
        .execution = CHASSIS_EXECUTION_LEGACY_CONTROLLER,
        .implemented = true,
        .compute = legacy_swerve_compute
    };
    return &strategy;
}
