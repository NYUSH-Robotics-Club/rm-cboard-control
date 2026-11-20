/*
 * 声明 Seasky 帧头的 CRC8 计算函数。
 * 输入长度单位为字节。
 */
#ifndef CRC8_H
#define CRC8_H

#include <stdint.h>

#define CRC_START_8 0x00

/**
 * @brief 计算CRC8校验码
 * @param input_str 输入数据
 * @param num_bytes 数据长度
 * @return CRC8校验码
 */
uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);

/**
 * @brief 更新CRC8校验码
 * @param crc 当前CRC值
 * @param val 新数据字节
 * @return 更新后的CRC8校验码
 */
uint8_t update_crc_8(uint8_t crc, uint8_t val);

#endif // CRC8_H
