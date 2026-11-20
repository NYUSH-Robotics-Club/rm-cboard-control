/* Ensure every shared message still fits in the fixed message-center payload. */
#include <assert.h>
#include <stdint.h>
#include "can_messages.h"
#include "control_messages.h"
#include "message_center.h"
#include "motor_messages.h"
#include "remote_messages.h"
#include "sensor_messages.h"
#include "vision_messages.h"

int main(void)
{
    assert(sizeof(VisionTargetMessage) <= MC_MAX_PAYLOAD);
    assert(sizeof(VisionRobotStateMessage) <= MC_MAX_PAYLOAD);
    assert(sizeof(CanRxFrame) <= MC_MAX_PAYLOAD);
    assert(sizeof(RemoteControlMessage) <= MC_MAX_PAYLOAD);
    assert(sizeof(SensorData) <= MC_MAX_PAYLOAD);
    assert(sizeof(MotorFeedbackEvent) <= MC_MAX_PAYLOAD);
    assert(sizeof(ChassisCmd) <= MC_MAX_PAYLOAD);
    MsgEvent event;
    assert(((uintptr_t)event.data % 4U) == 0U);
    return 0;
}
