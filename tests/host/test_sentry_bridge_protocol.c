#include "sentry_bridge_protocol.h"

#include <assert.h>
#include <math.h>
#include <string.h>

static void write_f32(unsigned char *p, float value)
{
    memcpy(p, &value, sizeof(value));
}

int main(void)
{
    unsigned char sx[SENTRY_BRIDGE_SX_SIZE] = {0};
    SentryBridgeCommand command;
    sx[0] = 'S';
    sx[1] = 'X';
    write_f32(sx + 2, 0.4f);
    write_f32(sx + 6, -0.3f);
    write_f32(sx + 10, 1.2f);
    write_f32(sx + 14, 0.0f);
    write_f32(sx + 18, 0.0f);
    sx[22] = 0x11;
    write_f32(sx + 23, 90.0f);
    write_f32(sx + 27, NAN);
    uint16_t crc = SentryBridge_Crc16X25(sx, SENTRY_BRIDGE_SX_SIZE - 2U);
    sx[31] = (unsigned char)crc;
    sx[32] = (unsigned char)(crc >> 8);

    assert(SentryBridge_ParseSx(sx, sizeof(sx), &command, 100U));
    assert(fabsf(command.vx - 0.4f) < 1e-6f);
    assert(fabsf(command.vy + 0.3f) < 1e-6f);
    assert(command.control_flags == 0x11U);
    assert(isnan(command.search_pitch_deg));

    sx[32] ^= 1U;
    assert(!SentryBridge_ParseSx(sx, sizeof(sx), &command, 101U));

    SentryBridgeTelemetry telemetry = {
        .cmd_vx = 0.4f, .cmd_vy = -0.3f, .cmd_wz = 1.2f,
        .real_vx = 0.3f, .real_vy = -0.2f, .real_wz = 1.0f,
        .robot_status = 3U, .game_status = 4U, .stage_remain_time = 220U,
        .robot_id = 107U, .current_hp = 600U, .shooter_heat = 35U,
        .team_color = 1U, .is_attacked = 1U,
    };
    unsigned char st[SENTRY_BRIDGE_ST_SIZE];
    assert(SentryBridge_BuildSt(&telemetry, st, sizeof(st)) == sizeof(st));
    assert(st[0] == 'S' && st[1] == 'T');
    assert(SentryBridge_Crc16X25(st, sizeof(st) - 2U) ==
           ((uint16_t)st[37] | ((uint16_t)st[38] << 8)));
    return 0;
}
