#include "diagnosis_can.h"
#include "can.h"
#include <string.h>

#define RX_CAPACITY 64U
#define TX_TIMEOUT_MS 50U
typedef struct {
    CAN_HandleTypeDef *handle;
    volatile DiagnosisBus stats;
    DiagnosisEvent tx;
    uint32_t mailbox, submitted_ms;
    bool timed_out;
} Port;
static Port ports[2];
static DiagnosisEvent rx_queue[RX_CAPACITY];
static volatile uint32_t rx_write, rx_read;

static uint32_t lock_irq(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}
static void unlock_irq(uint32_t primask) { __set_PRIMASK(primask); }

bool DiagnosisCan_Configure(uint32_t channel, uint32_t bitrate, uint32_t mode)
{
    if (channel < 1 || channel > 2 || mode > DIAG_LOOPBACK ||
        (bitrate != 125000 && bitrate != 200000 && bitrate != 250000 &&
         bitrate != 500000 && bitrate != 1000000)) return false;
    Port *port = &ports[channel - 1];
    if (port->stats.pending) return false;
    CAN_HandleTypeDef *hcan = channel == 1 ? &hcan1 : &hcan2;
    port->handle = hcan;
    HAL_NVIC_DisableIRQ(channel == 1 ? CAN1_RX0_IRQn : CAN2_RX0_IRQn);
    if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_LISTENING && HAL_CAN_Stop(hcan) != HAL_OK) {
        port->stats.ready = 0;
        return false;
    }
    port->stats.ready = 0;
    /* APB1 is 36 MHz; 12 time quanta per bit, from the frozen clock tree.
     * No retries: a missing ACK produces one failed attempt, never a stale
     * motor command that suddenly transmits after wiring is repaired.
     */
    hcan->Init.Prescaler = 3000000U / bitrate;
    hcan->Init.Mode = mode == DIAG_SILENT ? CAN_MODE_SILENT :
                       mode == DIAG_LOOPBACK ? CAN_MODE_SILENT_LOOPBACK : CAN_MODE_NORMAL;
    hcan->Init.AutoRetransmission = DISABLE;
    hcan->Init.AutoBusOff = DISABLE;
    if (HAL_CAN_Init(hcan) != HAL_OK) return false;
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = channel == 1 ? 0 : 14;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK || HAL_CAN_Start(hcan) != HAL_OK ||
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK)
        return false;
    port->stats.bitrate = bitrate;
    port->stats.mode = mode;
    port->stats.ready = 1;
    HAL_NVIC_EnableIRQ(channel == 1 ? CAN1_RX0_IRQn : CAN2_RX0_IRQn);
    return true;
}

/* Both RX IRQs have the same priority and cannot preempt each other. This
 * single producer queue is private to BSP; SWD never edits its read cursor.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t channel = hcan == &hcan1 ? 1 : 2;
    Port *port = &ports[channel - 1];
    port->stats.rx_callbacks++;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)) {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8] = {0};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
        port->stats.rx_frames++;
        port->stats.last_rx_ms = HAL_GetTick();
        if ((uint32_t)(rx_write - rx_read) >= RX_CAPACITY) {
            port->stats.rx_dropped++;
            continue;
        }
        DiagnosisEvent event = {.ms = HAL_GetTick(), .kind = DIAG_RX, .channel = channel,
            .id = header.IDE == CAN_ID_STD ? header.StdId : header.ExtId,
            .flags = (header.IDE == CAN_ID_EXT ? DIAG_EXTENDED : 0U) |
                     (header.RTR == CAN_RTR_REMOTE ? DIAG_REMOTE : 0U), .length = header.DLC};
        memcpy(event.data, data, 8);
        rx_queue[rx_write % RX_CAPACITY] = event;
        __DMB();
        rx_write++;
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetError(hcan) & HAL_CAN_ERROR_RX_FOV0)
        ports[hcan == &hcan1 ? 0 : 1].stats.rx_dropped++;
}

uint32_t DiagnosisCan_Send(const DiagnosisRequest *request, uint32_t now_ms)
{
    Port *port = &ports[request->channel - 1];
    if (!port->stats.ready) return DIAG_HARDWARE;
    if (port->stats.mode == DIAG_SILENT) return DIAG_LISTEN_ONLY;
    if (port->stats.pending) return DIAG_BUSY;
    CAN_HandleTypeDef *hcan = port->handle;
    if (hcan->Instance->ESR & CAN_ESR_BOFF) return DIAG_HARDWARE;
    CAN_TxHeaderTypeDef header = {.StdId = request->id, .ExtId = request->id,
        .IDE = (request->flags & DIAG_EXTENDED) ? CAN_ID_EXT : CAN_ID_STD,
        .RTR = (request->flags & DIAG_REMOTE) ? CAN_RTR_REMOTE : CAN_RTR_DATA,
        .DLC = request->length, .TransmitGlobalTime = DISABLE};
    uint8_t data[8];
    memcpy(data, request->data, 8);
    if (HAL_CAN_AddTxMessage(hcan, &header, data, &port->mailbox) != HAL_OK) return DIAG_HARDWARE;
    port->stats.pending = 1;
    port->stats.tx_queued++;
    port->submitted_ms = now_ms;
    port->timed_out = false;
    port->tx = (DiagnosisEvent){.channel = request->channel, .id = request->id,
        .flags = request->flags, .length = request->length,
        .data = {request->data[0], request->data[1]}, .token = request->seq};
    return DIAG_OK;
}

bool DiagnosisCan_Poll(DiagnosisEvent *event, uint32_t now_ms)
{
    for (unsigned i = 0; i < 2; ++i) {
        Port *port = &ports[i];
        if (!port->stats.pending) continue;
        unsigned shift = port->mailbox == CAN_TX_MAILBOX0 ? 0 :
                         port->mailbox == CAN_TX_MAILBOX1 ? 8 : 16;
        uint32_t tsr = port->handle->Instance->TSR;
        uint32_t done = CAN_TSR_RQCP0 << shift;
        if (tsr & done) {
            port->stats.last_tx_tsr = tsr;
            *event = port->tx;
            event->ms = now_ms;
            event->kind = (tsr & (CAN_TSR_TXOK0 << shift)) ? DIAG_TX_OK : DIAG_TX_ERROR;
            if (event->kind == DIAG_TX_OK) port->stats.tx_ok++;
            else port->stats.tx_error++;
            /* TX IRQs stay disabled; polling owns the completion bits so
             * mailbox acceptance cannot be confused with an ACKed CAN frame.
             */
            port->handle->Instance->TSR = done;
            port->stats.pending = 0;
            return true;
        }
        if (!port->timed_out && (uint32_t)(now_ms - port->submitted_ms) >= TX_TIMEOUT_MS) {
            port->stats.last_tx_tsr = tsr;
            port->timed_out = true;
            port->stats.tx_timeout++;
            HAL_CAN_AbortTxRequest(port->handle, port->mailbox);
            *event = port->tx;
            event->ms = now_ms;
            event->kind = DIAG_TX_TIMEOUT;
            /* Keep ownership until hardware confirms cancellation/completion.
             * If cancellation fails, further sends/configuration stay BUSY.
             */
            return true;
        }
    }
    uint32_t primask = lock_irq();
    bool available = rx_write != rx_read;
    if (available) {
        __DMB();
        *event = rx_queue[rx_read % RX_CAPACITY];
        rx_read++;
    }
    unlock_irq(primask);
    return available;
}

void DiagnosisCan_Snapshot(uint32_t channel, DiagnosisBus *bus)
{
    Port *port = &ports[channel - 1];
    uint32_t primask = lock_irq();
    *bus = port->stats;
    if (port->handle) {
        bus->esr = port->handle->Instance->ESR;
        bus->tsr = port->handle->Instance->TSR;
        bus->rf0r = port->handle->Instance->RF0R;
    }
    unlock_irq(primask);
}
