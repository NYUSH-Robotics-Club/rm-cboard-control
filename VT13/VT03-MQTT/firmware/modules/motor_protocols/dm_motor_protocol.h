/* 达妙 MIT 模式的纯编解码接口，不执行 CAN 收发。 */
#ifndef DM_MOTOR_PROTOCOL_H
#define DM_MOTOR_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float position_min;
    float position_max;
    float velocity_min;
    float velocity_max;
    float torque_min;
    float torque_max;
} DmMotorLimits;

typedef struct {
    uint8_t state;
    float position;
    float velocity;
    float torque;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
} DmMotorFeedback;

bool DmMotorProtocol_EncodeMit(const DmMotorLimits *limits,
                               float position,
                               float velocity,
                               float kp,
                               float kd,
                               float torque,
                               uint8_t data[8]);
bool DmMotorProtocol_DecodeFeedback(const DmMotorLimits *limits,
                                    const uint8_t data[8],
                                    DmMotorFeedback *feedback);
void DmMotorProtocol_BuildStateCommand(uint8_t command, uint8_t data[8]);

#endif
