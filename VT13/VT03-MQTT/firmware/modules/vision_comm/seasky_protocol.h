/*
 * 声明旧 Seasky 帧的组装和解析函数。
 * 这是传输协议，不应直接作为应用层消息。
 */
#ifndef SEASKY_PROTOCOL_H
#define SEASKY_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>

#define PROTOCOL_CMD_ID 0xA5
#define OFFSET_BYTE 8  // 帧头(4) + cmd_id(2) + 帧尾(2) = 8字节

typedef struct
{
    struct
    {
        uint8_t sof;
        uint16_t data_length;
        uint8_t crc_check;  // 帧头CRC校验
    } header;               // 数据帧头
    uint16_t cmd_id;        // 数据ID
    uint16_t frame_tail;    // 帧尾CRC校验
} protocol_rm_struct;

/**
 * @brief 更新发送数据帧，并计算发送数据帧长度
 * @param send_id 信号id
 * @param flags_register 16位寄存器
 * @param tx_data 待发送的float数据
 * @param float_length float的数据长度
 * @param tx_buf 待发送的数据帧
 * @param tx_buf_len 待发送的数据帧长度
 */
void get_protocol_send_data(uint16_t send_id,
                            uint16_t flags_register,
                            float *tx_data,
                            uint8_t float_length,
                            uint8_t *tx_buf,
                            uint16_t *tx_buf_len);

/**
 * @brief 接收数据处理
 * @param rx_buf 接收到的原始数据
 * @param flags_register 接收数据的16位寄存器地址
 * @param rx_data 接收的float数据存储地址
 * @return 返回cmd_id，校验失败返回0
 */
uint16_t get_protocol_info(uint8_t *rx_buf,
                           uint16_t *flags_register,
                           uint8_t *rx_data);

#endif // SEASKY_PROTOCOL_H
