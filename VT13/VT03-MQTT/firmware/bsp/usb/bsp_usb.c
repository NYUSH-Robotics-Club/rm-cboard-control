/* 当前通过 STM32 USB CDC 发送；busy 或错误时返回 false。 */
#include "bsp_usb.h"
#include "usbd_cdc_if.h"

bool BspUsb_Write(const uint8_t *data, uint16_t length)
{
    if (!data || length == 0U) {
        return false;
    }
    return CDC_Transmit_FS((uint8_t *)data, length) == USBD_OK;
}
