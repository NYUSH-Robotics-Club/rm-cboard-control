/*
 * Defines the decoded remote-control message used by robot applications.
 * Raw SBUS bytes and UART/DMA details stay in the remote module and BSP.
 */
#ifndef REMOTE_MESSAGES_H
#define REMOTE_MESSAGES_H

#include <stdint.h>

#define RC_CH_VALUE_MIN    ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX    ((uint16_t)1684)

#define RC_SW_UP   ((uint16_t)1)
#define RC_SW_MID  ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

#define switch_is_down(value) ((value) == RC_SW_DOWN)
#define switch_is_mid(value)  ((value) == RC_SW_MID)
#define switch_is_up(value)   ((value) == RC_SW_UP)

#define KEY_PRESSED_OFFSET_W     ((uint16_t)1U << 0)
#define KEY_PRESSED_OFFSET_S     ((uint16_t)1U << 1)
#define KEY_PRESSED_OFFSET_A     ((uint16_t)1U << 2)
#define KEY_PRESSED_OFFSET_D     ((uint16_t)1U << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1U << 4)
#define KEY_PRESSED_OFFSET_CTRL  ((uint16_t)1U << 5)
#define KEY_PRESSED_OFFSET_Q     ((uint16_t)1U << 6)
#define KEY_PRESSED_OFFSET_E     ((uint16_t)1U << 7)
#define KEY_PRESSED_OFFSET_R     ((uint16_t)1U << 8)
#define KEY_PRESSED_OFFSET_F     ((uint16_t)1U << 9)
#define KEY_PRESSED_OFFSET_G     ((uint16_t)1U << 10)
#define KEY_PRESSED_OFFSET_Z     ((uint16_t)1U << 11)
#define KEY_PRESSED_OFFSET_X     ((uint16_t)1U << 12)
#define KEY_PRESSED_OFFSET_C     ((uint16_t)1U << 13)
#define KEY_PRESSED_OFFSET_V     ((uint16_t)1U << 14)
#define KEY_PRESSED_OFFSET_B     ((uint16_t)1U << 15)

typedef struct {
    int16_t ch[5];
    int8_t s[2];
} RemoteChannels;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
} RemoteMouse;

typedef struct {
    uint16_t v;
} RemoteKeyboard;

typedef struct {
    RemoteChannels rc;
    RemoteMouse mouse;
    RemoteKeyboard key;
} RemoteControlMessage;

/* Compatibility name retained for existing startup and device modules. */
typedef RemoteControlMessage RC_ctrl_t;

#endif
