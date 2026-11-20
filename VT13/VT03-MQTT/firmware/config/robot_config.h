/*
 * 根据 CMake 的 ROBOT_TYPE 选择唯一机器人配置。
 * 名称错误时编译立即失败，避免使用错误车型参数。
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "config_types.h"

/**
 * Robot Configuration Selector
 *
 * This file selects the appropriate robot configuration based on
 * compile-time definitions set by CMake:
 *
 *   cmake -S . -B build -DROBOT_TYPE=infantry_standard
 *   cmake -S . -B build -DROBOT_TYPE=sentry_swerve
 */

#if defined(ROBOT_TYPE_infantry_standard)
  #include "infantry_standard.h"
  #define ACTIVE_ROBOT_CONFIG g_robot_config_infantry_standard

#elif defined(ROBOT_TYPE_sentry_swerve)
  #include "sentry_swerve.h"
  #define ACTIVE_ROBOT_CONFIG g_robot_config_sentry_swerve
#else
  #error "Unknown or missing ROBOT_TYPE_*. Set -DROBOT_TYPE=infantry_standard|sentry_swerve"
#endif

/**
 * @brief Get the active robot configuration
 * @return Pointer to the active robot configuration structure
 */
const RobotConfig_t* RobotConfig_Get(void);

#endif // ROBOT_CONFIG_H
