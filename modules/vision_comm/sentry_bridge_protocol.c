#include "sentry_bridge_protocol.h"

#include <math.h>
#include <string.h>

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static float read_f32(const uint8_t *data)
{
    float value;
    memcpy(&value, data, sizeof(value));
    return value;
}

static void write_f32(uint8_t *data, float value)
{
    memcpy(data, &value, sizeof(value));
}

uint16_t SentryBridge_Crc16X25(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    if (!data) return 0U;
    for (size_t i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0x8408U)
                             : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

bool SentryBridge_ParseSx(const uint8_t *frame, size_t length,
                          SentryBridgeCommand *command, uint32_t now_ms)
{
    if (!frame || !command || length != SENTRY_BRIDGE_SX_SIZE ||
        frame[0] != 'S' || frame[1] != 'X') {
        return false;
    }
    if (SentryBridge_Crc16X25(frame, length - 2U) != read_u16(frame + length - 2U)) {
        return false;
    }

    SentryBridgeCommand parsed = {
        .vx = read_f32(frame + 2U),
        .vy = read_f32(frame + 6U),
        .wz = read_f32(frame + 10U),
        .gimbal_yaw_delta = read_f32(frame + 14U),
        .gimbal_pitch_delta = read_f32(frame + 18U),
        .control_flags = frame[22],
        .scan_yaw_rate_deg_s = read_f32(frame + 23U),
        .search_pitch_deg = read_f32(frame + 27U),
        .received_ms = now_ms,
        .valid = true,
    };
    if (!isfinite(parsed.vx) || !isfinite(parsed.vy) || !isfinite(parsed.wz) ||
        !isfinite(parsed.gimbal_yaw_delta) || !isfinite(parsed.gimbal_pitch_delta) ||
        !isfinite(parsed.scan_yaw_rate_deg_s) ||
        (!isfinite(parsed.search_pitch_deg) && !isnan(parsed.search_pitch_deg))) {
        return false;
    }
    *command = parsed;
    return true;
}

size_t SentryBridge_BuildSt(const SentryBridgeTelemetry *telemetry,
                            uint8_t *frame, size_t capacity)
{
    if (!telemetry || !frame || capacity < SENTRY_BRIDGE_ST_SIZE) return 0U;
    frame[0] = 'S';
    frame[1] = 'T';
    write_f32(frame + 2U, telemetry->cmd_vx);
    write_f32(frame + 6U, telemetry->cmd_vy);
    write_f32(frame + 10U, telemetry->cmd_wz);
    write_f32(frame + 14U, telemetry->real_vx);
    write_f32(frame + 18U, telemetry->real_vy);
    write_f32(frame + 22U, telemetry->real_wz);
    frame[26] = telemetry->robot_status;
    frame[27] = telemetry->game_status;
    write_u16(frame + 28U, telemetry->stage_remain_time);
    frame[30] = telemetry->robot_id;
    write_u16(frame + 31U, telemetry->current_hp);
    write_u16(frame + 33U, telemetry->shooter_heat);
    frame[35] = telemetry->team_color;
    frame[36] = telemetry->is_attacked;
    write_u16(frame + 37U, SentryBridge_Crc16X25(frame, 37U));
    return SENTRY_BRIDGE_ST_SIZE;
}
