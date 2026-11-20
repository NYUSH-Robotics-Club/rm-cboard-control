/*
 * 集中声明各应用模块的初始化函数。
 * 启动代码只需调用这些入口，不需要了解模块内部订阅。
 */
#ifndef APP_SUBSCRIPTIONS_H
#define APP_SUBSCRIPTIONS_H

#include <stdint.h>
#include "robot_status.h"

/* Composition-facing application lifecycle API. */
void CmdController_Init(void);
void CmdController_Task(uint32_t now_ms);
void ChassisApp_Init(void);
void ShooterApp_Init(void);
void GimbalApp_Init(void);

/* Optional post-boot features are registered in runtime/app_manifest.c. */
RobotStatus AppRuntime_Init(void);
void AppRuntime_Step(uint32_t now_ms);
void AppRuntime_SafeStop(void);

#endif // APP_SUBSCRIPTIONS_H
