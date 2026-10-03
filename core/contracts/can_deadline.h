#ifndef CAN_DEADLINE_H
#define CAN_DEADLINE_H
#include <stdint.h>
/* Native-endian uint32 fields; telemetry serializes the same fixed layout. */
typedef struct {
    uint32_t software_expired, inflight_expired, abort_requests, abort_completed;
    /* abort_completed counts release without observed TXOK, not receiver acknowledgement. */
    uint32_t abort_failed, abort_raced_txok, max_submit_age_ms, max_inflight_age_ms;
    uint32_t max_abort_age_ms, last_id, last_slot, last_generated_ms;
    uint32_t event_sequence, last_event;
} BspCanDeadlineDiagnostics;
#endif
