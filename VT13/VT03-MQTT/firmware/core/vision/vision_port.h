/*
 * 定义视觉传输端口的统一操作。
 * 端口只负责收发标准消息，不包含识别算法或相机参数。
 */
#ifndef VISION_PORT_H
#define VISION_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_status.h"
#include "vision_messages.h"

typedef struct VisionPortOps {
    const char *name;
    bool implemented;
    RobotStatus (*init)(void *context);
    RobotStatus (*poll)(void *context);
    RobotStatus (*send_robot_state)(void *context,
                                    const VisionRobotStateMessage *state);
} VisionPortOps;

typedef struct {
    const VisionPortOps *ops;
    void *context;
} VisionPort;

/* Future Jetson transport slot. It remains unsupported until protocol details arrive. */
VisionPort JetsonVisionPort_Create(void);

#endif
