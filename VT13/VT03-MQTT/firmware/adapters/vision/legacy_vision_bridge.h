/*
 * 声明旧视觉协议桥接器的初始化入口。
 * 重复初始化是安全的，不会重复注册订阅。
 */
#ifndef LEGACY_VISION_BRIDGE_H
#define LEGACY_VISION_BRIDGE_H

#include "robot_status.h"

/* Normalize the current Seasky/USB topic into VisionTargetMessage. */
RobotStatus LegacyVisionBridge_Init(void);

#endif
