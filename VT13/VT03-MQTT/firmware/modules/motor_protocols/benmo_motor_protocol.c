/* BM1505B 命令是大端；反馈保留协议原始单位，型号换算由配置层决定。 */
#include "benmo_motor_protocol.h"
#include <stddef.h>

bool BenmoMotorProtocol_SetSlot(uint8_t motor_address,
                                uint8_t value,
                                uint8_t data[8])
{
    if (!data || motor_address < 1U || motor_address > 8U) return false;
    data[motor_address - 1U] = value;
    return true;
}

bool BenmoMotorProtocol_BuildGroup(const int16_t commands[4], uint8_t data[8])
{
    if (!commands || !data) return false;
    for (uint8_t slot = 0U; slot < 4U; ++slot) {
        const uint16_t raw = (uint16_t)commands[slot];
        data[slot * 2U] = (uint8_t)(raw >> 8);
        data[slot * 2U + 1U] = (uint8_t)raw;
    }
    return true;
}

bool BenmoMotorProtocol_DecodeFeedback(const uint8_t data[8],
                                       BenmoMotorFeedback *feedback)
{
    if (!data || !feedback) return false;
    feedback->velocity_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    feedback->current_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    feedback->angle_raw = (uint16_t)(((uint16_t)data[4] << 8) | data[5]);
    feedback->fault = data[6];
    feedback->mode = data[7];
    return true;
}
