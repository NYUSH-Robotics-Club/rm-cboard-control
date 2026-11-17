#include "diagnosis.h"
#include "diagnosis_can.h"
#include <string.h>

volatile DiagnosisShared g_diagnosis;
static uint32_t last_request;

static void record(DiagnosisEvent event)
{
    uint32_t seq = g_diagnosis.event_head + 1U;
    if (seq == 0U) seq = 1U;
    volatile DiagnosisEvent *slot = &g_diagnosis.events[(seq - 1U) % DIAG_EVENT_COUNT];
    slot->seq = 0;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    event.seq = 0;
    *slot = event;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    slot->seq = seq;
    g_diagnosis.event_head = seq;
}

void Diagnosis_Init(void)
{
    memset((void *)&g_diagnosis, 0, sizeof(g_diagnosis));
    last_request = 0;
    g_diagnosis.abi = DIAG_ABI;
    g_diagnosis.bytes = sizeof(g_diagnosis);
    /* Passive boot: no CAN transmissions or acknowledgements until configured. */
    DiagnosisCan_Configure(1, 1000000, DIAG_SILENT);
    DiagnosisCan_Configure(2, 1000000, DIAG_SILENT);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    g_diagnosis.magic = DIAG_MAGIC;
}

void Diagnosis_Step(uint32_t now_ms)
{
    g_diagnosis.version++;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    g_diagnosis.uptime_ms = now_ms;
    g_diagnosis.loops++;
    DiagnosisEvent event;
    /* Bounded work: a saturated bus cannot starve host commands. */
    for (unsigned n = 0; n < 64 && DiagnosisCan_Poll(&event, now_ms); ++n) record(event);

    uint32_t seq = g_diagnosis.request.seq;
    if (seq != 0 && seq != last_request) {
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        DiagnosisRequest request = g_diagnosis.request;
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        if (seq == g_diagnosis.request.seq) {
            uint32_t result = DIAG_INVALID;
            if (request.channel >= 1 && request.channel <= 2) {
                if (request.op == DIAG_CONFIGURE) {
                    DiagnosisBus bus;
                    DiagnosisCan_Snapshot(request.channel, &bus);
                    if (bus.pending) result = DIAG_BUSY;
                    else if ((request.arg0 == 125000 || request.arg0 == 200000 ||
                              request.arg0 == 250000 || request.arg0 == 500000 ||
                              request.arg0 == 1000000) && request.arg1 <= DIAG_LOOPBACK)
                        result = DiagnosisCan_Configure(request.channel, request.arg0, request.arg1)
                                 ? DIAG_OK : DIAG_HARDWARE;
                } else if (request.op == DIAG_SEND && request.length <= 8 &&
                           request.flags <= (DIAG_EXTENDED | DIAG_REMOTE) &&
                           request.id <= ((request.flags & DIAG_EXTENDED) ? 0x1fffffffU : 0x7ffU)) {
                    result = DiagnosisCan_Send(&request, now_ms);
                    if (result == DIAG_OK) {
                        event = (DiagnosisEvent){.ms = now_ms, .kind = DIAG_TX_QUEUED,
                            .channel = request.channel, .id = request.id, .flags = request.flags,
                            .length = request.length, .data = {request.data[0], request.data[1]}, .token = seq};
                        record(event);
                    }
                }
            }
            last_request = seq;
            g_diagnosis.result = result;
            g_diagnosis.ack = seq;
        }
    }
    for (unsigned i = 0; i < 2; ++i) {
        DiagnosisBus bus;
        DiagnosisCan_Snapshot(i + 1, &bus);
        g_diagnosis.bus[i] = bus;
    }
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    g_diagnosis.version++;
}
