/* 瓴控 0x280 广播帧和 0x141~0x144 反馈均使用小端 16 位字段。 */
#include "lk_motor_protocol.h"
#include <stddef.h>

bool LkMotorProtocol_BuildBroadcastCurrent(const int16_t commands[4],
                                           uint8_t data[8])
{
    if (!commands || !data) return false;
    for (uint8_t slot = 0U; slot < 4U; ++slot) {
        const uint16_t raw = (uint16_t)commands[slot];
        data[slot * 2U] = (uint8_t)raw;
        data[slot * 2U + 1U] = (uint8_t)(raw >> 8);
    }
    return true;
}

bool LkMotorProtocol_DecodeFeedback(const uint8_t data[8],
                                    LkMotorFeedback *feedback)
{
    if (!data || !feedback) return false;
    feedback->temperature = data[1];
    feedback->current = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    feedback->speed_deg_s = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    feedback->encoder = (uint16_t)(((uint16_t)data[7] << 8) | data[6]);
    return true;
}
