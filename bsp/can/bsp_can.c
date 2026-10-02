/*
 * 实现当前 STM32 板卡的 CAN 启动、收发和诊断。
 * 滤波器分区沿用现有固件设置，不在业务模块里散布 HAL 调用。
 */
#include "bsp_can.h"
#include "main.h"
#include "bsp_critical.h"

#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static BspCanRecovery s_recovery[2];
static bool s_started[2];
static bool s_outputs_armed;
static bool s_healthy_seen;
static uint32_t s_healthy_since_ms;
static uint8_t s_abort_pending;

#define CAN_RECOVERY_STEP_TIMEOUT_MS 20U
#define CAN_RECOVERY_RETRY_MS 1000U
#define CAN_RECOVERY_STABLE_MS 500U
#define CAN_ALL_MAILBOXES (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)

static CAN_HandleTypeDef *handle_for(BspCanChannel channel)
{
    if (channel == BSP_CAN_CHANNEL_1) {
        return &hcan1;
    }
    if (channel == BSP_CAN_CHANNEL_2) {
        return &hcan2;
    }
    return NULL;
}

#define TRACE_CAPACITY 256U
static struct {
    uint32_t channel, tx_id, rx_id, slot, sequence, command_seq, lost;
    uint32_t head, tail;
    MotorTraceEvent ring[TRACE_CAPACITY];
    uint32_t mailbox_seq[3];
    bool mailbox_active[3];
} s_trace;

static uint32_t trace_push_at(uint32_t ms, uint32_t kind, int32_t raw, uint32_t detail)
{
    BspCriticalState lock = BspCritical_Enter();
    uint32_t seq = ++s_trace.sequence;
    if (s_trace.head - s_trace.tail == TRACE_CAPACITY) {
        ++s_trace.tail;
        ++s_trace.lost;
    }
    s_trace.ring[s_trace.head++ % TRACE_CAPACITY] =
        (MotorTraceEvent){ms, seq, kind, raw, detail};
    BspCritical_Exit(lock);
    return seq;
}

static uint32_t trace_push(uint32_t kind, int32_t raw, uint32_t detail)
{
    return trace_push_at(HAL_GetTick(), kind, raw, detail);
}

/* Protocol decoding belongs to the caller; preserve its receive timestamp. */
void BspCan_TraceFeedback(BspCanChannel channel, uint16_t rx_id,
                          uint32_t ms, int32_t raw, uint32_t detail)
{
    if (s_trace.channel == (uint32_t)channel && s_trace.rx_id == rx_id)
        trace_push_at(ms, MOTOR_TRACE_FEEDBACK, raw, detail);
}

void BspCan_TraceConfigure(BspCanChannel channel, uint16_t tx_id, uint16_t rx_id, uint8_t slot)
{
    if (!handle_for(channel) || slot >= 4U) return;
    memset(&s_trace, 0, sizeof(s_trace));
    s_trace.channel = channel; s_trace.tx_id = tx_id;
    s_trace.rx_id = rx_id; s_trace.slot = slot;
}

void BspCan_TraceCommand(BspCanChannel channel, uint16_t tx_id, uint8_t slot, int16_t raw)
{
    if (s_trace.channel == (uint32_t)channel && s_trace.tx_id == tx_id && s_trace.slot == slot)
        s_trace.command_seq = trace_push(MOTOR_TRACE_COMMAND, raw, 0U);
}

/* TX IRQ remains disabled. Observe RQCP/TXOK/ALST/TERR before HAL reuses an
 * empty mailbox (reuse invalidates its old result). Never clear or alter TSR here.
 * A completion without a retained result is UNKNOWN, never assumed successful. */
static void trace_poll(BspCanChannel channel)
{
    if (s_trace.channel != (uint32_t)channel) return;
    uint32_t tsr = handle_for(channel)->Instance->TSR;
    for (unsigned i = 0U; i < 3U; ++i) {
        if (!s_trace.mailbox_active[i]) continue;
        uint32_t bits = (tsr >> (8U * i)) & 15U;
        if ((bits & 1U) == 0U && (tsr & (1UL << (26U + i))) == 0U) continue;
        uint32_t kind = MOTOR_TRACE_TX_UNKNOWN;
        if (bits & 1U) {
            if (bits & 2U) kind = MOTOR_TRACE_TX_OK;
            else if (bits & 4U) kind = MOTOR_TRACE_TX_ARB_LOST;
            else if (bits & 8U) kind = MOTOR_TRACE_TX_ERROR;
        }
        trace_push(kind | ((i + 1U) << 8), (int32_t)tsr, s_trace.mailbox_seq[i]);
        s_trace.mailbox_active[i] = false;
    }
}

void BspCan_TraceRead(MotorTraceBatch *batch)
{
    memset(batch, 0, sizeof(*batch));
    if (!s_trace.channel) return;
    trace_poll((BspCanChannel)s_trace.channel);
    batch->channel = s_trace.channel; batch->tx_id = s_trace.tx_id;
    batch->rx_id = s_trace.rx_id; batch->slot = s_trace.slot;
    /* Bound interrupt masking to one event, not a whole telemetry packet. */
    for (unsigned i = 0U; i < MOTOR_TRACE_BATCH; ++i) {
        BspCriticalState lock = BspCritical_Enter();
        if (s_trace.tail == s_trace.head) { BspCritical_Exit(lock); break; }
        batch->events[batch->count++] = s_trace.ring[s_trace.tail++ % TRACE_CAPACITY];
        BspCritical_Exit(lock);
    }
    batch->lost = s_trace.lost;
    CAN_HandleTypeDef *handle = handle_for((BspCanChannel)s_trace.channel);
    batch->esr = handle->Instance->ESR;
    batch->tsr = handle->Instance->TSR;
    batch->hal_error = HAL_CAN_GetError(handle);
    batch->free_mailboxes = HAL_CAN_GetTxMailboxesFreeLevel(handle);
}

bool BspCan_Start(BspCanChannel channel)
{
    CAN_HandleTypeDef *handle = handle_for(channel);
    if (!handle || !handle->Instance) {
        return false;
    }

    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = channel == BSP_CAN_CHANNEL_1 ? 0U : 14U;
    /* Both controllers share CAN2SB: preserve CAN1 banks when starting CAN2. */
    filter.SlaveStartFilterBank = 14U;
    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;

    if (HAL_CAN_ConfigFilter(handle, &filter) != HAL_OK) {
        return false;
    }
    if (HAL_CAN_Start(handle) != HAL_OK) {
        return false;
    }
    if (HAL_CAN_ActivateNotification(handle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return false;
    }
    unsigned index = (unsigned)channel - 1U;
    memset(&s_recovery[index], 0, sizeof(s_recovery[index]));
    s_started[index] = true;
    s_outputs_armed = false;
    s_healthy_seen = false;
    return true;
}

/* Abort both buses so an old command on the healthy bus cannot survive the stop. */
static void latch_bus_fault(unsigned index, uint32_t now_ms)
{
    BspCanRecovery *state = &s_recovery[index];
    CAN_HandleTypeDef *handle = handle_for((BspCanChannel)(index + 1U));
    if (state->phase != 0U) return;
    state->phase = 1U;
    state->phase_since_ms = now_ms;
    state->fault_count++;
    state->first_fault_esr = handle->Instance->ESR;
    s_outputs_armed = false;
    s_healthy_seen = false;
    for (unsigned i = 0U; i < 2U; ++i) {
        if (s_started[i]) {
            s_abort_pending |= (uint8_t)(1U << i);
            (void)HAL_CAN_AbortTxRequest(handle_for((BspCanChannel)(i + 1U)), CAN_ALL_MAILBOXES);
        }
    }
}

void BspCan_Service(uint32_t now_ms)
{
    bool healthy = s_started[0] && s_started[1];
    for (unsigned i = 0U; i < 2U; ++i) {
        if (!s_started[i]) continue;
        CAN_HandleTypeDef *handle = handle_for((BspCanChannel)(i + 1U));
        BspCanRecovery *state = &s_recovery[i];
        uint32_t esr = handle->Instance->ESR;
        if ((esr & CAN_ESR_LEC) != 0U) state->last_error_esr = esr;
        if ((esr & CAN_ESR_BOFF) != 0U && state->phase == 0U) latch_bus_fault(i, now_ms);
        if ((s_abort_pending & (1U << i)) != 0U && HAL_CAN_GetTxMailboxesFreeLevel(handle) == 3U) {
            s_abort_pending &= (uint8_t)~(1U << i);
        }
        uint32_t elapsed = (uint32_t)(now_ms - state->phase_since_ms);
        switch (state->phase) {
        case 1U:
            /* Do not leave a nonzero mailbox pending across reinitialization. */
            if (HAL_CAN_GetTxMailboxesFreeLevel(handle) == 3U) {
                SET_BIT(handle->Instance->MCR, CAN_MCR_INRQ);
                state->phase = 2U;
                state->phase_since_ms = now_ms;
            } else if (elapsed >= CAN_RECOVERY_STEP_TIMEOUT_MS) {
                state->phase = 5U;
                state->phase_since_ms = now_ms;
                state->timeout_count++;
            }
            break;
        case 2U:
            if ((handle->Instance->MSR & CAN_MSR_INAK) != 0U) {
                /* RM0090: set then clear INRQ with ABOM=0; hardware does bus synchronization. */
                CLEAR_BIT(handle->Instance->MCR, CAN_MCR_INRQ);
                state->phase = 3U;
                state->phase_since_ms = now_ms;
            } else if (elapsed >= CAN_RECOVERY_STEP_TIMEOUT_MS) {
                state->phase = 5U;
                state->phase_since_ms = now_ms;
                state->timeout_count++;
            }
            break;
        case 3U:
            if ((handle->Instance->MSR & CAN_MSR_INAK) == 0U) {
                state->phase = 4U;
                state->phase_since_ms = now_ms;
            } else if (elapsed >= CAN_RECOVERY_RETRY_MS) {
                state->phase = 5U;
                state->phase_since_ms = now_ms;
                state->timeout_count++;
            }
            break;
        case 4U:
            if ((esr & CAN_ESR_BOFF) == 0U) {
                state->phase = 0U;
                state->recovery_count++;
            } else if (elapsed >= CAN_RECOVERY_RETRY_MS) {
                state->phase = 5U;
                state->phase_since_ms = now_ms;
                state->timeout_count++;
            }
            break;
        case 5U:
            if (elapsed >= CAN_RECOVERY_RETRY_MS) {
                (void)HAL_CAN_AbortTxRequest(handle, CAN_ALL_MAILBOXES);
                state->phase = 1U;
                state->phase_since_ms = now_ms;
            }
            break;
        default:
            break;
        }
        if (state->phase != 0U || (esr & (CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF)) != 0U) {
            healthy = false;
        }
    }
    if (s_abort_pending != 0U) healthy = false;
    if (!healthy) s_healthy_seen = false;
    else if (!s_healthy_seen) {
        s_healthy_seen = true;
        s_healthy_since_ms = now_ms;
    }
}

const BspCanRecovery *BspCan_GetRecovery(BspCanChannel channel)
{
    return (channel == BSP_CAN_CHANNEL_1 || channel == BSP_CAN_CHANNEL_2)
        ? &s_recovery[(unsigned)channel - 1U] : NULL;
}

bool BspCan_OutputsArmed(void) { return s_outputs_armed; }

bool BspCan_RecoveryReady(uint32_t now_ms)
{
    return s_healthy_seen && (uint32_t)(now_ms - s_healthy_since_ms) >= CAN_RECOVERY_STABLE_MS;
}

bool BspCan_TryArm(uint32_t now_ms)
{
    if (!BspCan_RecoveryReady(now_ms)) return false;
    for (unsigned i = 0U; i < 2U; ++i) {
        CAN_HandleTypeDef *handle = handle_for((BspCanChannel)(i + 1U));
        if (s_recovery[i].phase != 0U ||
            (handle->Instance->ESR & (CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF)) != 0U) return false;
    }
    s_outputs_armed = true;
    return true;
}

bool BspCan_Read(BspCanChannel channel, BspCanFrame *frame)
{
    CAN_HandleTypeDef *handle = handle_for(channel);
    if (!handle || !handle->Instance || !frame) {
        return false;
    }

    CAN_RxHeaderTypeDef header;
    uint8_t data[8] = {0};
    if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, data) != HAL_OK) {
        return false;
    }

    frame->standard_id = header.StdId;
    frame->length = header.DLC <= 8U ? (uint8_t)header.DLC : 8U;
    memcpy(frame->data, data, frame->length);
    frame->is_standard_frame = header.IDE == CAN_ID_STD;
    frame->is_data_frame = header.RTR == CAN_RTR_DATA;
    return true;
}

static bool write_frame(BspCanChannel channel,
                  uint16_t standard_id,
                  const uint8_t *data,
                  uint8_t length)
{
    CAN_HandleTypeDef *handle = handle_for(channel);
    if (!handle || !handle->Instance || length > 8U || (length > 0U && !data)) {
        return false;
    }

    unsigned index = (unsigned)channel - 1U;
    if (!s_started[index]) return false;
    if ((handle->Instance->ESR & CAN_ESR_BOFF) != 0U) {
        latch_bus_fault(index, HAL_GetTick());
        return false;
    }
    if (s_recovery[index].phase != 0U || (s_abort_pending & (1U << index)) != 0U) return false;
    /* No buffered nonzero command may escape while the operator interlock is closed. */
    if (!s_outputs_armed) {
        for (uint8_t i = 0U; i < length; ++i) if (data[i] != 0U) return false;
    }

    CAN_TxHeaderTypeDef header = {0};
    uint8_t frame_data[8] = {0};
    uint32_t mailbox;
    header.StdId = standard_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = length;
    if (length > 0U) {
        memcpy(frame_data, data, length);
    }
    trace_poll(channel);
    bool ok = HAL_CAN_AddTxMessage(handle, &header, frame_data, &mailbox) == HAL_OK;
    if (ok && s_trace.channel == (uint32_t)channel) {
        for (unsigned i = 0U; i < 3U; ++i) if (mailbox == (1UL << i)) {
            uint32_t mailbox_tag = (i + 1U) << 8;
            /* Completion can race the pre-submit poll, including reuse by a
             * non-yaw frame. Never attribute that new frame's result to yaw. */
            if (s_trace.mailbox_active[i]) {
                trace_push(MOTOR_TRACE_TX_UNKNOWN | mailbox_tag, 0, s_trace.mailbox_seq[i]);
                s_trace.mailbox_active[i] = false;
            }
            if (standard_id == s_trace.tx_id && length == 8U) {
                unsigned slot = s_trace.slot * 2U;
                int16_t raw = (int16_t)((uint16_t)data[slot] << 8 | data[slot + 1U]);
                s_trace.mailbox_seq[i] = trace_push(MOTOR_TRACE_SUBMIT | mailbox_tag, raw, s_trace.command_seq);
                s_trace.mailbox_active[i] = true;
            }
        }
    }
    return ok;
}

bool BspCan_Write(BspCanChannel channel, uint16_t standard_id, const uint8_t *data, uint8_t length)
{
    bool ok = write_frame(channel, standard_id, data, length);
    if (!ok && s_trace.channel == (uint32_t)channel && standard_id == s_trace.tx_id && data && length == 8U) {
        unsigned slot = s_trace.slot * 2U;
        int16_t raw = (int16_t)((uint16_t)data[slot] << 8 | data[slot + 1U]);
        trace_push(MOTOR_TRACE_REJECT, raw, s_trace.command_seq);
    }
    return ok;
}

bool BspCan_MatchesNativeHandle(BspCanChannel channel, const void *native_handle)
{
    return native_handle && native_handle == handle_for(channel);
}

bool BspCan_ReadDiagnostics(BspCanChannel channel, BspCanDiagnostics *diagnostics)
{
    if (!diagnostics) {
        return false;
    }

    CAN_HandleTypeDef *handle = handle_for(channel);
    if (!handle || !handle->Instance) {
        return false;
    }

    diagnostics->error = HAL_CAN_GetError(handle);
    diagnostics->error_status_register = handle->Instance->ESR;
    diagnostics->transmit_status_register = handle->Instance->TSR;
    diagnostics->receive_fifo0_register = handle->Instance->RF0R;
    diagnostics->free_tx_mailboxes = HAL_CAN_GetTxMailboxesFreeLevel(handle);
    return true;
}
