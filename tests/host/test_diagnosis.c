#include "diagnosis.h"
#include "diagnosis_can.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static DiagnosisBus buses[2];
static unsigned sends, configurations;
static bool fail_config;
static DiagnosisEvent incoming;
static unsigned incoming_count;
bool DiagnosisCan_Configure(uint32_t channel, uint32_t bitrate, uint32_t mode)
{
    configurations++;
    if (fail_config) return false;
    buses[channel-1].bitrate = bitrate;
    buses[channel-1].mode = mode;
    return true;
}
uint32_t DiagnosisCan_Send(const DiagnosisRequest *request, uint32_t now)
{
    (void)now;
    DiagnosisBus *bus = &buses[request->channel-1];
    if (bus->mode == DIAG_SILENT) return DIAG_LISTEN_ONLY;
    if (bus->pending) return DIAG_BUSY;
    sends++;
    return DIAG_OK;
}
bool DiagnosisCan_Poll(DiagnosisEvent *event, uint32_t now)
{
    (void)now;
    if (!incoming_count) return false;
    *event = incoming;
    incoming_count--;
    return true;
}
void DiagnosisCan_Snapshot(uint32_t channel, DiagnosisBus *bus) { *bus = buses[channel-1]; }
static unsigned seq;
static void request(DiagnosisRequest r, uint32_t expected)
{
    r.seq = ++seq;
    g_diagnosis.request = r;
    Diagnosis_Step(seq * 2);
    assert(g_diagnosis.ack == seq && g_diagnosis.result == expected);
    assert((g_diagnosis.version & 1) == 0);
}
int main(void)
{
    Diagnosis_Init();
    assert(configurations == 2 && sends == 0);
    assert(g_diagnosis.magic == DIAG_MAGIC && sizeof(g_diagnosis) == 5376);
    request((DiagnosisRequest){.op=DIAG_SEND,.channel=1,.id=1,.length=8}, DIAG_LISTEN_ONLY);
    request((DiagnosisRequest){.op=DIAG_CONFIGURE,.channel=0,.arg0=1000000,.arg1=1}, DIAG_INVALID);
    request((DiagnosisRequest){.op=DIAG_CONFIGURE,.channel=1,.arg0=2000000,.arg1=1}, DIAG_INVALID);
    request((DiagnosisRequest){.op=DIAG_CONFIGURE,.channel=1,.arg0=1000000,.arg1=1}, DIAG_OK);
    request((DiagnosisRequest){.op=DIAG_SEND,.channel=1,.id=0x800,.length=8}, DIAG_INVALID);
    request((DiagnosisRequest){.op=DIAG_SEND,.channel=1,.id=1,.length=9}, DIAG_INVALID);
    request((DiagnosisRequest){.op=DIAG_SEND,.channel=1,.id=1,.flags=4}, DIAG_INVALID);
    request((DiagnosisRequest){.op=DIAG_SEND,.channel=1,.id=0x7ff,.length=4,.data={0x08330001}}, DIAG_OK);
    assert(sends == 1 && g_diagnosis.event_head == 1);
    assert(g_diagnosis.events[0].kind == DIAG_TX_QUEUED && g_diagnosis.events[0].token == seq);
    Diagnosis_Step(100);
    assert(sends == 1); /* Re-reading an acknowledged request never retransmits. */
    buses[0].pending=1;
    request((DiagnosisRequest){.op=DIAG_SEND,.channel=1,.id=1}, DIAG_BUSY);
    request((DiagnosisRequest){.op=DIAG_CONFIGURE,.channel=1,.arg0=500000,.arg1=1}, DIAG_BUSY);
    buses[0].pending=0;
    fail_config=true;
    request((DiagnosisRequest){.op=DIAG_CONFIGURE,.channel=2,.arg0=500000,.arg1=1}, DIAG_HARDWARE);
    fail_config=false;
    request((DiagnosisRequest){.op=DIAG_SEND,.channel=1,.id=0x1fffffff,.flags=1,.length=0}, DIAG_OK);
    request((DiagnosisRequest){.op=DIAG_SEND,.channel=1,.id=0x20000000,.flags=1}, DIAG_INVALID);
    incoming=(DiagnosisEvent){.kind=DIAG_RX,.channel=2,.id=0,.length=8,.data={0x12345678,0xabcdef01},.ms=120};
    incoming_count=140;
    Diagnosis_Step(122); assert(incoming_count == 76);
    Diagnosis_Step(124); Diagnosis_Step(126);
    assert(g_diagnosis.event_head == 142);
    DiagnosisEvent last=g_diagnosis.events[(142-1)%128];
    assert(last.seq==142 && last.data[1]==0xabcdef01 && last.channel==2);
    g_diagnosis.event_head=0xffffffffU;
    incoming_count=1;
    Diagnosis_Step(128);
    assert(g_diagnosis.event_head==1 && g_diagnosis.events[0].seq==1);
    puts("diagnosis service: PASS");
}
