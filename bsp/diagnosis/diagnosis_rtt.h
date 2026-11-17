#ifndef DIAGNOSIS_RTT_H
#define DIAGNOSIS_RTT_H
#include <stdbool.h>
#include <stddef.h>
void DiagnosisRtt_Init(void);
size_t DiagnosisRtt_Read(void *data, size_t length);
bool DiagnosisRtt_Write(const void *data, size_t length);
#endif
