#ifndef SENTRY_BRIDGE_PROTOCOL_H
#define SENTRY_BRIDGE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SENTRY_BRIDGE_SX_SIZE 33U
#define SENTRY_BRIDGE_ST_SIZE 39U
#define SENTRY_BRIDGE_CMD_TIMEOUT_MS 250U

typedef struct {
    float vx;
    float vy;
    float wz;
    float gimbal_yaw_delta;
    float gimbal_pitch_delta;
    uint8_t control_flags;
    float scan_yaw_rate_deg_s;
    float search_pitch_deg;
    uint32_t received_ms;
    bool valid;
} SentryBridgeCommand;

typedef struct {
    float cmd_vx;
    float cmd_vy;
    float cmd_wz;
    float real_vx;
    float real_vy;
    float real_wz;
    uint8_t robot_status;
    uint8_t game_status;
    uint16_t stage_remain_time;
    uint8_t robot_id;
    uint16_t current_hp;
    uint16_t shooter_heat;
    uint8_t team_color;
    uint8_t is_attacked;
} SentryBridgeTelemetry;

uint16_t SentryBridge_Crc16X25(const uint8_t *data, size_t length);
bool SentryBridge_ParseSx(const uint8_t *frame, size_t length,
                           SentryBridgeCommand *command, uint32_t now_ms);
size_t SentryBridge_BuildSt(const SentryBridgeTelemetry *telemetry,
                            uint8_t *frame, size_t capacity);

#endif
