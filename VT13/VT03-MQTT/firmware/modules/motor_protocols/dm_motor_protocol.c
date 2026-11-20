/* 按达妙 MIT 标准帧排列 16 位位置和四个 12 位字段。 */
#include "dm_motor_protocol.h"
#include <math.h>
#include <stddef.h>

static bool valid_range(float low, float high)
{
    return isfinite(low) && isfinite(high) && low < high;
}

static float clamp_float(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static uint16_t float_to_uint(float value, float low, float high, uint8_t bits)
{
    const uint32_t maximum = (1UL << bits) - 1UL;
    const float normalized = (clamp_float(value, low, high) - low) / (high - low);
    return (uint16_t)(normalized * (float)maximum + 0.5f);
}

static float uint_to_float(uint16_t value, float low, float high, uint8_t bits)
{
    const uint32_t maximum = (1UL << bits) - 1UL;
    return ((float)value * (high - low) / (float)maximum) + low;
}

static bool limits_valid(const DmMotorLimits *limits)
{
    return limits &&
           valid_range(limits->position_min, limits->position_max) &&
           valid_range(limits->velocity_min, limits->velocity_max) &&
           valid_range(limits->torque_min, limits->torque_max);
}

bool DmMotorProtocol_EncodeMit(const DmMotorLimits *limits,
                               float position,
                               float velocity,
                               float kp,
                               float kd,
                               float torque,
                               uint8_t data[8])
{
    if (!limits_valid(limits) || !data || !isfinite(position) ||
        !isfinite(velocity) || !isfinite(kp) || !isfinite(kd) ||
        !isfinite(torque)) {
        return false;
    }

    const uint16_t p = float_to_uint(position, limits->position_min,
                                     limits->position_max, 16U);
    const uint16_t v = float_to_uint(velocity, limits->velocity_min,
                                     limits->velocity_max, 12U);
    const uint16_t kp_raw = float_to_uint(kp, 0.0f, 500.0f, 12U);
    const uint16_t kd_raw = float_to_uint(kd, 0.0f, 5.0f, 12U);
    const uint16_t t = float_to_uint(torque, limits->torque_min,
                                     limits->torque_max, 12U);

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)((v << 4) | (kp_raw >> 8));
    data[4] = (uint8_t)kp_raw;
    data[5] = (uint8_t)(kd_raw >> 4);
    data[6] = (uint8_t)((kd_raw << 4) | (t >> 8));
    data[7] = (uint8_t)t;
    return true;
}

bool DmMotorProtocol_DecodeFeedback(const DmMotorLimits *limits,
                                    const uint8_t data[8],
                                    DmMotorFeedback *feedback)
{
    if (!limits_valid(limits) || !data || !feedback) {
        return false;
    }
    const uint16_t p = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    const uint16_t v = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    const uint16_t t = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | data[5]);

    feedback->state = (uint8_t)(data[0] >> 4);
    feedback->position = uint_to_float(p, limits->position_min,
                                       limits->position_max, 16U);
    feedback->velocity = uint_to_float(v, limits->velocity_min,
                                       limits->velocity_max, 12U);
    feedback->torque = uint_to_float(t, limits->torque_min,
                                     limits->torque_max, 12U);
    feedback->mos_temperature = data[6];
    feedback->rotor_temperature = data[7];
    return true;
}

void DmMotorProtocol_BuildStateCommand(uint8_t command, uint8_t data[8])
{
    if (!data) return;
    for (uint8_t index = 0U; index < 7U; ++index) {
        data[index] = 0xFFU;
    }
    data[7] = command;
}
