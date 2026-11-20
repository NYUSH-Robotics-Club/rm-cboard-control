/*
 * 根据机器人配置选择麦轮、舵轮或全向轮策略。
 * 未知类型返回空指针，调用方必须停止输出。
 */
#include "chassis_strategy.h"
#include "robot_config.h"

const ChassisStrategy *ChassisStrategy_Get(ChassisType_e type)
{
    switch (type) {
        case CHASSIS_TYPE_MECANUM:
            return MecanumChassisStrategy_Get();
        case CHASSIS_TYPE_SWERVE:
            return SwerveChassisStrategy_Get();
        case CHASSIS_TYPE_OMNI:
            return OmniChassisStrategy_Get();
        default:
            return 0;
    }
}

const ChassisStrategy *ChassisStrategy_GetActive(void)
{
    const RobotConfig_t *config = RobotConfig_Get();
    return config ? ChassisStrategy_Get(config->chassis_type) : 0;
}
