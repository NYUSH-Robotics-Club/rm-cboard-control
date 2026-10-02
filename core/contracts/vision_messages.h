/*
 * 定义视觉适配器与应用之间的稳定消息。
 * 角度使用弧度，距离使用米，可选字段必须通过标志说明是否有效。
 */
#ifndef VISION_MESSAGES_H
#define VISION_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>

#define VISION_TARGET_SCHEMA_VERSION 1U

typedef enum {
    VISION_FIRE_NONE = 0,
    VISION_FIRE_AUTO = 1,
    VISION_AIM_ONLY = 2
} VisionFireMode;

typedef enum {
    VISION_TARGET_NONE = 0,
    VISION_TARGET_CONVERGING = 1,
    VISION_TARGET_READY = 2
} VisionTargetState;

typedef enum {
    VISION_FIELD_DISTANCE = (1U << 0),
    VISION_FIELD_VELOCITY = (1U << 1),
    VISION_FIELD_CONFIDENCE = (1U << 2),
    VISION_FIELD_SOURCE_TIMESTAMP = (1U << 3)
} VisionFieldFlags;

/*
 * Stable in-process contract between a vision adapter and robot applications.
 * It is deliberately not a packed wire protocol. Future Jetson transports must
 * decode their protocol into this type before publishing it.
 */
typedef struct {
    uint16_t schema_version;
    uint16_t field_flags;
    uint32_t sequence;
    uint32_t source_timestamp_ms;
    /* 相对接收时轴位置的角度误差；正值增加该轴编码目标，非摇杆量。
     * 相机坐标到电机编码方向的转换由发送端/适配器负责，须实车核对。 */
    float yaw_error_rad;
    float pitch_error_rad;
    float distance_m;
    float target_yaw_rate_rad_s;
    float target_pitch_rate_rad_s;
    float confidence;
    uint8_t target_id;
    uint8_t fire_mode;
    uint8_t target_state;
    bool valid;
} VisionTargetMessage;

typedef struct {
    uint16_t schema_version;
    uint16_t field_flags;
    uint32_t sequence;
    uint32_t timestamp_ms;
    float yaw_rad;
    float pitch_rad;
    float roll_rad;
    float yaw_rate_rad_s;
    float pitch_rate_rad_s;
    float roll_rate_rad_s;
} VisionRobotStateMessage;

#endif
