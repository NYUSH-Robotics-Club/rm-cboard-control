/* 瓴控广播电流模式的纯编解码接口，不包含车型控制逻辑。 */
#ifndef LK_MOTOR_PROTOCOL_H
#define LK_MOTOR_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t temperature;
    int16_t current;
    int16_t speed_deg_s;
    uint16_t encoder;
} LkMotorFeedback;

bool LkMotorProtocol_BuildBroadcastCurrent(const int16_t commands[4],
                                           uint8_t data[8]);
bool LkMotorProtocol_DecodeFeedback(const uint8_t data[8],
                                    LkMotorFeedback *feedback);

#endif
