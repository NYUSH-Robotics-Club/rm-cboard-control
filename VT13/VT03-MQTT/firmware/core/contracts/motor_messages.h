/*
 * Defines motor feedback messages and the latest feedback state used by
 * controllers. CAN frame layouts remain inside the CAN and motor adapters.
 */
#ifndef MOTOR_MESSAGES_H
#define MOTOR_MESSAGES_H

#include <stdint.h>

typedef struct {
    uint8_t id;
    uint16_t angle;
    int16_t speed;
    int16_t current;
    uint8_t temp;
    uint32_t tick_ms;
} MotorFeedbackEvent;

typedef struct {
    uint8_t id;
    uint16_t angle;
    int16_t speed;
    uint32_t tick_ms;
    int16_t current;
} GM6020FeedbackEvent;

typedef struct {
    uint16_t angle;
    int16_t speed;
    int16_t current;
    uint8_t temp;
    uint32_t last_update_time;
} MotorFeedbackState;

/* Compatibility name retained for existing controller structures. */
typedef MotorFeedbackState Motor_Feedback;

#endif
