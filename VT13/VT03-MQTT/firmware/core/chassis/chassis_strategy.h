/*
 * 定义不同底盘共用的输入、输出和选择接口。
 * 输入为归一化车体指令，策略本身不访问电机或消息中心。
 */
#ifndef CHASSIS_STRATEGY_H
#define CHASSIS_STRATEGY_H

#include <stdbool.h>
#include <stdint.h>
#include "config_types.h"
#include "robot_status.h"

#define CHASSIS_MAX_DRIVE_MOTORS 4U
#define CHASSIS_MAX_STEER_MOTORS 4U

typedef enum {
    CHASSIS_EXECUTION_KINEMATICS = 0,
    CHASSIS_EXECUTION_LEGACY_CONTROLLER,
    CHASSIS_EXECUTION_PLACEHOLDER
} ChassisExecutionMode;

typedef struct {
    float vx;
    float vy;
    float wz;
    float max_drive_speed; /* Absolute rotor RPM ceiling, shared desaturation. */
    const OmniChassisConfig *omni; /* Required only by the omni strategy. */
} ChassisKinematicsInput;

typedef struct {
    float drive_speed[CHASSIS_MAX_DRIVE_MOTORS]; /* Rotor RPM before motor sign. */
    float steer_angle[CHASSIS_MAX_STEER_MOTORS];
    uint8_t drive_count;
    uint8_t steer_count;
} ChassisKinematicsOutput;

typedef struct ChassisStrategy {
    ChassisType_e type;
    const char *name;
    ChassisExecutionMode execution;
    bool implemented;
    RobotStatus (*compute)(const ChassisKinematicsInput *input,
                           ChassisKinematicsOutput *output);
} ChassisStrategy;

const ChassisStrategy *ChassisStrategy_Get(ChassisType_e type);
const ChassisStrategy *ChassisStrategy_GetActive(void);
const ChassisStrategy *MecanumChassisStrategy_Get(void);
const ChassisStrategy *SwerveChassisStrategy_Get(void);
const ChassisStrategy *OmniChassisStrategy_Get(void);

#endif
