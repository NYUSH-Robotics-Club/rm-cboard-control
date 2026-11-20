/*
 * 声明 Seasky 协议使用的 CRC16 计算函数。
 * 输入长度单位为字节。
 */
#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

#define CRC_START_16 0xFFFF
#define CRC_POLY_16  0xA001

/**
 * @brief 计算CRC16校验码
 * @param input_str 输入数据
 * @param num_bytes 数据长度
 * @return CRC16校验码
 */
uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes);

/**
 * @brief 更新CRC16校验码
 * @param crc 当前CRC值
 * @param c 新数据字节
 * @return 更新后的CRC16校验码
 */
uint16_t update_crc_16(uint16_t crc, uint8_t c);

/**
 * @brief 初始化CRC16查找表
 */
void init_crc16_tab(void);

#endif // CRC16_H
