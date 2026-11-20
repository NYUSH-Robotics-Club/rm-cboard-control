/*
 * Jetson 视觉通信的安全占位入口。
 * 传输和消息格式未知时不收发数据，并返回“不支持”。
 */
#include "vision_port.h"

static RobotStatus jetson_unsupported_init(void *context)
{
    (void)context;
    return ROBOT_STATUS_UNSUPPORTED;
}

static RobotStatus jetson_unsupported_poll(void *context)
{
    (void)context;
    return ROBOT_STATUS_UNSUPPORTED;
}

static RobotStatus jetson_unsupported_send(
    void *context,
    const VisionRobotStateMessage *state)
{
    (void)context;
    (void)state;
    return ROBOT_STATUS_UNSUPPORTED;
}

VisionPort JetsonVisionPort_Create(void)
{
    static const VisionPortOps ops = {
        .name = "Jetson placeholder (transport/protocol required)",
        .implemented = false,
        .init = jetson_unsupported_init,
        .poll = jetson_unsupported_poll,
        .send_robot_state = jetson_unsupported_send
    };
    VisionPort port = {.ops = &ops, .context = 0};
    return port;
}
