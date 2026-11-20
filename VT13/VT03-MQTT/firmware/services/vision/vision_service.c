/* 当前启用旧 Seasky 桥；未来 Jetson 协议只在这里更换或并行登记。 */
#include "vision_service.h"
#include "legacy_vision_bridge.h"

RobotStatus VisionService_Init(void)
{
    return LegacyVisionBridge_Init();
}
