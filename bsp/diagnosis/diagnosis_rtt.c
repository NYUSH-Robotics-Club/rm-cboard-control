#include "diagnosis_rtt.h"
#include "SEGGER_RTT.h"
void DiagnosisRtt_Init(void) { SEGGER_RTT_Init(); }
size_t DiagnosisRtt_Read(void *data, size_t length) { return SEGGER_RTT_Read(0, data, (unsigned)length); }
bool DiagnosisRtt_Write(const void *data, size_t length)
{
    /* Whole records only; no blocking wait may hold up CAN service. RTT is
     * used exclusively in foreground, never from a receive interrupt.
     */
    return SEGGER_RTT_Write(0, data, (unsigned)length) == length;
}
