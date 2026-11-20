/*
 * 把旧 Seasky 视觉数据转换成应用层统一消息。
 * 应用控制器只读取统一消息，不依赖 USB 帧格式。
 */
#include "legacy_vision_bridge.h"
#include "message_center.h"
#include "vision_comm.h"
#include "vision_messages.h"

static bool s_initialized = false;
static uint32_t s_sequence = 0U;

static void on_legacy_vision(const MsgEvent *event, void *user_data)
{
    (void)user_data;
    if (!event || event->size != sizeof(Vision_Recv_s)) {
        return;
    }

    const Vision_Recv_s *legacy = (const Vision_Recv_s *)event->data;
    VisionTargetMessage target = {
        .schema_version = VISION_TARGET_SCHEMA_VERSION,
        .field_flags = 0U,
        .sequence = ++s_sequence,
        .source_timestamp_ms = 0U,
        .yaw_error_rad = legacy->yaw,
        .pitch_error_rad = legacy->pitch,
        .distance_m = 0.0f,
        .target_yaw_rate_rad_s = 0.0f,
        .target_pitch_rate_rad_s = 0.0f,
        .confidence = 0.0f,
        .target_id = (uint8_t)legacy->target_type,
        .fire_mode = (uint8_t)legacy->fire_mode,
        .target_state = (uint8_t)legacy->target_state,
        .valid = legacy->updated && legacy->target_state != NO_TARGET
    };
    (void)MsgCenter_Publish(TOPIC_VISION_TARGET, &target, sizeof(target));
}

RobotStatus LegacyVisionBridge_Init(void)
{
    if (s_initialized) {
        return ROBOT_STATUS_OK;
    }
    int result = MsgCenter_Subscribe(TOPIC_VISION_DATA,
                                     on_legacy_vision,
                                     NULL);
    if (result != 0) {
        return ROBOT_STATUS_CAPACITY_EXCEEDED;
    }
    s_initialized = true;
    return ROBOT_STATUS_OK;
}
