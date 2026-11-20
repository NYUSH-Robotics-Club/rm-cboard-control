/*
 * 提供板级 UART 字节发送接口。
 * native_handle 只用于兼容冻结启动代码传入的旧句柄。
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

bool BspUart6_Write(const uint8_t *data, uint16_t length);
bool BspUart_WriteNative(void *native_handle,
                         const uint8_t *data,
                         uint16_t length,
                         uint32_t timeout_ms);

#endif
