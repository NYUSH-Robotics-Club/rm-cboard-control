/* Observe and route the copied nyush-rm-control USART service.
 * Decoding and watchdog state belong to the upstream remote/daemon modules. */
#ifndef BSP_REMOTE_BRIDGE_H
#define BSP_REMOTE_BRIDGE_H
#include <stdint.h>
#define BSP_RC_FRAME_BYTES 18U
typedef void (*BspRcFrameCallback)(const uint8_t *frame, uint16_t length);
/* Live counters may change during a read. They do not drive control decisions. */
typedef struct {
    uint32_t rx_events, idle_events, dma_complete_events, received_bytes;
    uint32_t last_packet_bytes, uart_errors, last_uart_error, start_attempts;
    uint32_t busy_starts, failed_starts, last_hal_status, warning_count, log_error_count;
} BspRcDiagnostics;
const volatile BspRcDiagnostics *BspRc_GetDiagnostics(void);
/* Called after the upstream decoder; packet snapshot is valid during this call only. */
void BspRc_SetFrameCallback(BspRcFrameCallback callback);
/* USART3 branch of the shared HAL RxEvent callback. Length is in bytes. */
void BspRc_OnRxEvent(uint16_t length);
/* VT02/VT03 UART input, independent of the existing USART3 DBUS receiver. */
void BspRc_VtmInit(void);
int BspRc_VtmReadByte(void);

#endif
