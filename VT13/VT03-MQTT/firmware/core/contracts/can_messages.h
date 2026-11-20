/* 原始 CAN 帧的跨层消息；携带通道和时间，订阅者无需访问 HAL 句柄。 */
#ifndef CAN_MESSAGES_H
#define CAN_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>
#include "config_types.h"

typedef struct {
    CAN_Channel_t channel;
    uint16_t std_id;
    uint8_t dlc;
    bool is_standard_frame;
    bool is_data_frame;
    uint8_t data[8];
    uint32_t tick_ms;
} CanRxFrame;

#endif
