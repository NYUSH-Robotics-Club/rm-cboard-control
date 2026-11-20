/* Bridge the copied nyush-rm-control USART service to this project's HAL callback.
 * Counters are observational; receive/retry decisions stay in the upstream source. */
#include "bsp_rc.h"
#include "main.h"
#include "vtm_dma_cursor.h"
#include <string.h>
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
#ifndef VR_VTM_UART_BAUD
#define VR_VTM_UART_BAUD 115200U
#endif
static volatile uint8_t vtm_started;
#if VR_VTM_UART_BAUD == 921600U
/* USART6_RX is DMA2 Stream1/channel5 (RM0090). This unused stream is
 * configured here so the frozen CubeMX files and other DMA users stay intact. */
static DMA_HandleTypeDef vtm_dma;
static uint8_t vtm_ring[VTM_DMA_SIZE];
static volatile uint32_t vtm_wraps;
static uint32_t vtm_tail, vtm_dma_base;
void DMA2_Stream1_IRQHandler(void) { HAL_DMA_IRQHandler(&vtm_dma); }
#else
static uint8_t vtm_byte, vtm_ring[256];
static volatile uint8_t vtm_head, vtm_tail;
#endif
volatile uint32_t vr_uart_rx_bytes;
volatile uint32_t vr_uart_ring_overflows;
volatile uint32_t vr_uart_errors;
volatile uint32_t vr_uart_last_error;
volatile uint32_t vr_uart_actual_baud;

void BspRc_VtmInit(void)
{
    /* UART1 on the C-board connector is USART6. Set its transport rate here,
     * leaving the generated UART configuration and the DBUS receiver intact. */
    huart6.Init.BaudRate = VR_VTM_UART_BAUD;
    if (HAL_UART_Init(&huart6) != HAL_OK) return;
    vr_uart_actual_baud = huart6.Init.BaudRate;
    __HAL_UART_CLEAR_OREFLAG(&huart6);
#if VR_VTM_UART_BAUD == 921600U
    __HAL_RCC_DMA2_CLK_ENABLE();
    vtm_dma.Instance = DMA2_Stream1;
    vtm_dma.Init.Channel = DMA_CHANNEL_5;
    vtm_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    vtm_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    vtm_dma.Init.MemInc = DMA_MINC_ENABLE;
    vtm_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    vtm_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    vtm_dma.Init.Mode = DMA_CIRCULAR;
    vtm_dma.Init.Priority = DMA_PRIORITY_HIGH;
    vtm_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&vtm_dma) != HAL_OK) return;
    __HAL_LINKDMA(&huart6, hdmarx, vtm_dma);
    HAL_NVIC_DisableIRQ(USART6_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    if (HAL_UART_Receive_DMA(&huart6, vtm_ring, VTM_DMA_SIZE) != HAL_OK) return;
    __HAL_DMA_DISABLE_IT(&vtm_dma, DMA_IT_HT);
    vtm_started = 1;
#else
    vtm_started = 1;
    (void)HAL_UART_Receive_IT(&huart6, &vtm_byte, 1);
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart6 || !vtm_started) return;
#if VR_VTM_UART_BAUD == 921600U
    ++vtm_wraps;
#else
    ++vr_uart_rx_bytes;
    uint8_t next = (uint8_t)(vtm_head + 1U);
    if (next != vtm_tail) {
        vtm_ring[vtm_head] = vtm_byte;
        __DMB();
        vtm_head = next;
    } else ++vr_uart_ring_overflows;
    (void)HAL_UART_Receive_IT(uart, &vtm_byte, 1);
#endif
}

int BspRc_VtmReadByte(void)
{
#if VR_VTM_UART_BAUD == 921600U
    if (!vtm_started) return -1;
    uint32_t errors = huart6.Instance->SR &
        (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE);
    if (errors) {
        ++vr_uart_errors;
        vr_uart_last_error = errors;
        __HAL_UART_CLEAR_OREFLAG(&huart6);
    }
    /* A pending TC flag means hardware wrapped before its ISR ran. Account
     * for it while this stream's IRQ is masked; the second flag read catches
     * a wrap between reading NDTR and the first flag. No global IRQ masking. */
    HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
    uint32_t wraps = vtm_wraps;
    uint32_t pending = __HAL_DMA_GET_FLAG(&vtm_dma, DMA_FLAG_TCIF1_5);
    uint32_t remaining = __HAL_DMA_GET_COUNTER(&vtm_dma);
    if (__HAL_DMA_GET_FLAG(&vtm_dma, DMA_FLAG_TCIF1_5)) {
        if (!pending) remaining = __HAL_DMA_GET_COUNTER(&vtm_dma);
        ++wraps;
    }
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    uint32_t producer = VtmDma_Producer(wraps, remaining);
    /* A reload observed just before its TC flag may briefly look older than
     * the last consumed byte. Retry next poll rather than declaring overflow. */
    if ((int32_t)(producer - vtm_tail) < 0) return -1;
    vr_uart_rx_bytes = vtm_dma_base + producer;
    uint32_t available = producer - vtm_tail;
    if (available > VTM_DMA_SIZE) {
        vr_uart_ring_overflows += available - VTM_DMA_SIZE;
        vtm_tail = producer - VTM_DMA_SIZE;
    }
    if (producer == vtm_tail) return -1;
    __DMB();
    return vtm_ring[vtm_tail++ & (VTM_DMA_SIZE - 1U)];
#else
    if (vtm_started && huart6.RxState == HAL_UART_STATE_READY)
        (void)HAL_UART_Receive_IT(&huart6, &vtm_byte, 1);
    uint8_t tail = vtm_tail;
    if (tail == vtm_head) return -1;
    __DMB();
    int byte = vtm_ring[tail];
    __DMB();
    vtm_tail = (uint8_t)(tail + 1U);
    return byte;
#endif
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
        ++vr_uart_errors;
        vr_uart_last_error = uart->ErrorCode;
        /* ORE can remain set after RXNE was consumed. HAL's IRQ path then
         * ends reception without reading DR; rearming alone retriggers ORE
         * indefinitely. The documented SR/DR read clears that latched error. */
        __HAL_UART_CLEAR_OREFLAG(uart);
#if VR_VTM_UART_BAUD == 921600U
        /* DMA transfer errors end reception in HAL. Restart this channel only;
         * a broken frame will be discarded by the existing CRC parser. */
        if (uart->RxState == HAL_UART_STATE_READY) {
            vtm_dma_base = vr_uart_rx_bytes;
            vtm_wraps = 0;
            vtm_tail = 0;
            if (HAL_UART_Receive_DMA(uart, vtm_ring, VTM_DMA_SIZE) == HAL_OK)
                __HAL_DMA_DISABLE_IT(&vtm_dma, DMA_IT_HT);
        }
#else
        if (uart->RxState == HAL_UART_STATE_READY)
            (void)HAL_UART_Receive_IT(uart, &vtm_byte, 1);
#endif
        return;
    }
    if (uart != &huart3) return;
    s_diagnostics.uart_errors++;
    s_diagnostics.last_uart_error = uart->ErrorCode;
    BspRc_UpstreamError(uart);
}
