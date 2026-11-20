/* 本末 BM1505B 分组命令、配置和原始反馈编解码接口。 */
#ifndef BENMO_MOTOR_PROTOCOL_H
#define BENMO_MOTOR_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define BENMO_MODE_FRAME_ID          0x105U
#define BENMO_FEEDBACK_MODE_FRAME_ID 0x106U
#define BENMO_CONTROL_GROUP_1_ID     0x032U
#define BENMO_CONTROL_GROUP_2_ID     0x033U

typedef struct {
    int16_t velocity_raw;
    int16_t current_raw;
    uint16_t angle_raw;
    uint8_t fault;
    uint8_t mode;
} BenmoMotorFeedback;

bool BenmoMotorProtocol_SetSlot(uint8_t motor_address,
                                uint8_t value,
                                uint8_t data[8]);
bool BenmoMotorProtocol_BuildGroup(const int16_t commands[4], uint8_t data[8]);
bool BenmoMotorProtocol_DecodeFeedback(const uint8_t data[8],
                                       BenmoMotorFeedback *feedback);

#endif
