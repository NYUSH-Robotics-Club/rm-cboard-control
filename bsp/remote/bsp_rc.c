/* Bridge the copied nyush-rm-control USART service to this project's HAL callback.
 * Counters are observational; receive/retry decisions stay in the upstream source. */
#include "bsp_rc.h"
#include "main.h"
#include <string.h>
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
static uint8_t vtm_byte, vtm_ring[256];
static volatile uint8_t vtm_head, vtm_tail;
static volatile uint8_t vtm_started;

void BspRc_VtmInit(void)
{
    vtm_started = 1;
    (void)HAL_UART_Receive_IT(&huart6, &vtm_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart6 || !vtm_started) return;
    uint8_t next = (uint8_t)(vtm_head + 1U);
    if (next != vtm_tail) {
        vtm_ring[vtm_head] = vtm_byte;
        __DMB();
        vtm_head = next;
    }
    (void)HAL_UART_Receive_IT(uart, &vtm_byte, 1);
}

int BspRc_VtmReadByte(void)
{
    if (vtm_started && huart6.RxState == HAL_UART_STATE_READY)
        (void)HAL_UART_Receive_IT(&huart6, &vtm_byte, 1);
    uint8_t tail = vtm_tail;
    if (tail == vtm_head) return -1;
    __DMB();
    int byte = vtm_ring[tail];
    __DMB();
    vtm_tail = (uint8_t)(tail + 1U);
    return byte;
}
void BspRc_UpstreamRxEvent(UART_HandleTypeDef *, uint16_t);
void BspRc_UpstreamError(UART_HandleTypeDef *);
static BspRcFrameCallback s_frame_callback;
static volatile BspRcDiagnostics s_diagnostics;
const volatile BspRcDiagnostics *BspRc_GetDiagnostics(void) { return &s_diagnostics; }
void BspRc_SetFrameCallback(BspRcFrameCallback cb) { s_frame_callback = cb; }
void BspRc_UpstreamWarning(void) { s_diagnostics.warning_count++; }
void BspRc_UpstreamLogError(void) { s_diagnostics.log_error_count++; }
HAL_StatusTypeDef BspRc_ObservedReceive(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(uart, buffer, size);
    if (uart == &huart3) {
        s_diagnostics.start_attempts++;
        s_diagnostics.last_hal_status = status;
        s_diagnostics.busy_starts += status == HAL_BUSY;
        s_diagnostics.failed_starts += status != HAL_OK && status != HAL_BUSY;
    }
    return status;
}
void BspRc_OnRxEvent(uint16_t length)
{
    uint8_t snapshot[BSP_RC_FRAME_BYTES];
    if (length == sizeof(snapshot)) memcpy(snapshot, huart3.pRxBuffPtr, sizeof(snapshot));
    HAL_UART_RxEventTypeTypeDef type = HAL_UARTEx_GetRxEventType(&huart3);
    s_diagnostics.rx_events++;
    s_diagnostics.idle_events += type == HAL_UART_RXEVENT_IDLE;
    s_diagnostics.dma_complete_events += type == HAL_UART_RXEVENT_TC;
    s_diagnostics.received_bytes += length;
    s_diagnostics.last_packet_bytes = length;
    BspRc_UpstreamRxEvent(&huart3, length);
    /* Upstream also decodes partial events. Preserve that upstream behavior,
     * but only complete DBUS frames may update this robot's motor commands. */
    if (length == sizeof(snapshot) && s_frame_callback) s_frame_callback(snapshot, length);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart6 && vtm_started) {
        /* HAL may have ended the reception after an overrun. */
        if (uart->RxState == HAL_UART_STATE_READY)
            (void)HAL_UART_Receive_IT(uart, &vtm_byte, 1);
        return;
    }
    if (uart != &huart3) return;
    s_diagnostics.uart_errors++;
    s_diagnostics.last_uart_error = uart->ErrorCode;
    BspRc_UpstreamError(uart);
}
