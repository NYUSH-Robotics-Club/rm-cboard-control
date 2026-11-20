/* 当前使用阻塞式 USART6 发送，仅供低频诊断文本。 */
#include "bsp_uart.h"
#include "main.h"

extern UART_HandleTypeDef huart6;

bool BspUart6_Write(const uint8_t *data, uint16_t length)
{
    return BspUart_WriteNative(&huart6, data, length, HAL_MAX_DELAY);
}

bool BspUart_WriteNative(void *native_handle,
                         const uint8_t *data,
                         uint16_t length,
                         uint32_t timeout_ms)
{
    if (!data || length == 0U) {
        return false;
    }
    UART_HandleTypeDef *handle = (UART_HandleTypeDef *)native_handle;
    if (!handle || !handle->Instance) {
        return false;
    }
    return HAL_UART_Transmit(handle, (uint8_t *)data, length, timeout_ms) == HAL_OK;
}
