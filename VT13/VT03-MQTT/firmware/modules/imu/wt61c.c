/*
 * 解析 WT61C 串口数据，并保存最新的加速度、角速度和姿态。
 * 字节发送交给 BSP，本文件不直接操作 UART 硬件。
 */

#include "wt61c.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include <string.h>

#ifndef GRAVITY
#define GRAVITY 9.80665f
#endif

static UART_HandleTypeDef *s_huart = NULL;
static WT61C_Data s_data;

typedef enum {
    ST_WAIT_55 = 0,
    ST_WAIT_ID,
    ST_PAYLOAD
} ParseState;

static struct {
    ParseState st;
    uint8_t buf[11];
    uint8_t idx;
} s_parser;

static int16_t to_s16(uint8_t lo, uint8_t hi);
static void parse_one_frame(const uint8_t *f);
static HAL_StatusTypeDef wit_cmd5(UART_HandleTypeDef *huart, uint8_t reg, uint16_t val);

static int16_t to_s16(uint8_t lo, uint8_t hi) {
    return (int16_t)((hi << 8) | lo);
}

static void parse_one_frame(const uint8_t *f) {
    uint8_t sum = 0;
    for (int i = 0; i < 10; ++i) {
        sum += f[i];
    }
    if (sum != f[10]) {
        return;
    }
    const uint8_t id = f[1];
    const uint8_t *p = &f[2];
    switch (id) {
    case 0x51: {
        int16_t ax_raw = to_s16(p[0], p[1]);
        int16_t ay_raw = to_s16(p[2], p[3]);
        int16_t az_raw = to_s16(p[4], p[5]);
        int16_t temp_raw = to_s16(p[6], p[7]);
        (void)temp_raw;
        s_data.ax = (ax_raw / 32768.0f) * 16.0f * GRAVITY;
        s_data.ay = (ay_raw / 32768.0f) * 16.0f * GRAVITY;
        s_data.az = (az_raw / 32768.0f) * 16.0f * GRAVITY;
        s_data.temperature = temp_raw / 100.0f;
        break;
    }
    case 0x52: {
        int16_t gx_raw = to_s16(p[0], p[1]);
        int16_t gy_raw = to_s16(p[2], p[3]);
        int16_t gz_raw = to_s16(p[4], p[5]);
        s_data.gx = (gx_raw / 32768.0f) * 2000.0f;
        s_data.gy = (gy_raw / 32768.0f) * 2000.0f;
        s_data.gz = (gz_raw / 32768.0f) * 2000.0f;
        break;
    }
    case 0x53: {
        int16_t roll_raw = to_s16(p[0], p[1]);
        int16_t pitch_raw = to_s16(p[2], p[3]);
        int16_t yaw_raw = to_s16(p[4], p[5]);
        s_data.roll = (roll_raw / 32768.0f) * 180.0f;
        s_data.pitch = (pitch_raw / 32768.0f) * 180.0f;
        s_data.yaw = (yaw_raw / 32768.0f) * 180.0f;
        break;
    }
    default:
        break;
    }
    s_data.last_update_ms = BspTime_NowMs();
}

void WT61C_ProcessBytes(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) {
        uint8_t b = data[i];
        switch (s_parser.st) {
        case ST_WAIT_55:
            if (b == 0x55) {
                s_parser.buf[0] = b;
                s_parser.idx = 1;
                s_parser.st = ST_WAIT_ID;
            }
            break;
        case ST_WAIT_ID:
            s_parser.buf[1] = b;
            s_parser.idx = 2;
            s_parser.st = ST_PAYLOAD;
            break;
        case ST_PAYLOAD:
            s_parser.buf[s_parser.idx++] = b;
            if (s_parser.idx >= 11) {
                parse_one_frame(s_parser.buf);
                s_parser.st = ST_WAIT_55;
                s_parser.idx = 0;
                WT61C_OnNewData(&s_data);
            }
            break;
        }
    }
}

void WT61C_Init(UART_HandleTypeDef *huart) {
    s_huart = huart;
    memset(&s_data, 0, sizeof(s_data));
    s_parser.st = ST_WAIT_55;
    s_parser.idx = 0;
}

const WT61C_Data* WT61C_GetData(void) {
    return &s_data;
}

static HAL_StatusTypeDef wit_cmd5(UART_HandleTypeDef *huart, uint8_t reg, uint16_t val) {
    uint8_t pkt[5] = { 0xFF, 0xAA, reg, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    return BspUart_WriteNative(huart, pkt, sizeof(pkt), 20U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef WT61C_Unlock(UART_HandleTypeDef *huart) {
    uint8_t unlock[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    return BspUart_WriteNative(huart, unlock, sizeof(unlock), 20U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef WT61C_SetReturnRate(UART_HandleTypeDef *huart, uint8_t rateEnum) {
    return wit_cmd5(huart, 0x03, rateEnum);
}

HAL_StatusTypeDef WT61C_Save(UART_HandleTypeDef *huart) {
    return wit_cmd5(huart, 0x00, 0x0000);
}

__weak void WT61C_OnNewData(const WT61C_Data *d) {
    (void)d;
}
