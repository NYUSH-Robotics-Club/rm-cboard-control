#ifndef DIAGNOSIS_CONTRACT_H
#define DIAGNOSIS_CONTRACT_H
#include <stdint.h>
#include <stddef.h>

#define DIAG_MAGIC 0x44494147U
#define DIAG_ABI 1U
#define DIAG_EVENT_COUNT 128U
enum { DIAG_CONFIGURE = 1, DIAG_SEND = 2 };
enum { DIAG_OK, DIAG_INVALID, DIAG_BUSY, DIAG_HARDWARE, DIAG_LISTEN_ONLY };
enum { DIAG_SILENT, DIAG_NORMAL, DIAG_LOOPBACK };
enum { DIAG_RX = 1, DIAG_TX_QUEUED, DIAG_TX_OK, DIAG_TX_ERROR, DIAG_TX_TIMEOUT };
enum { DIAG_EXTENDED = 1, DIAG_REMOTE = 2 };

/* Internal engine/debugger state, not the host transport. The RTT terminal
 * exclusively owns request; the engine owns status/events. IRQ records use
 * a private BSP queue. Bus/event word order is serialized by RTT protocol v2.
 */
typedef struct {
    uint32_t seq, ms, kind, channel, id, flags, length, data[2], token;
} DiagnosisEvent;
typedef struct {
    uint32_t bitrate, mode, ready, rx_frames, rx_dropped, rx_callbacks;
    uint32_t tx_queued, tx_ok, tx_error, tx_timeout;
    uint32_t esr, tsr, rf0r, last_tx_tsr, pending, last_rx_ms;
} DiagnosisBus;
typedef struct {
    uint32_t seq, op, channel, arg0, arg1, id, flags, length, data[2], reserved[6];
} DiagnosisRequest;
typedef struct {
    uint32_t magic, abi, bytes, version, uptime_ms, loops, event_head;
    uint32_t ack, result, reserved[7];
    DiagnosisBus bus[2];
    DiagnosisRequest request;
    DiagnosisEvent events[DIAG_EVENT_COUNT];
} DiagnosisShared;
_Static_assert(sizeof(DiagnosisBus) == 64, "RTT bus word order");
_Static_assert(sizeof(DiagnosisEvent) == 40, "RTT event word order");
_Static_assert(offsetof(DiagnosisShared, request) == 192, "SWD request ABI");
_Static_assert(offsetof(DiagnosisShared, events) == 256, "SWD events ABI");
#endif
