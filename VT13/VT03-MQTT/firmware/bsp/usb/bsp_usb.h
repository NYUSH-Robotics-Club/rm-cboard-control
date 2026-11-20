/* 提供不暴露 USB Device 实现细节的字节发送接口。 */
#ifndef BSP_USB_H
#define BSP_USB_H

#include <stdbool.h>
#include <stdint.h>

bool BspUsb_Write(const uint8_t *data, uint16_t length);

#endif
