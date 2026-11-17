#ifndef DIAGNOSIS_CAN_H
#define DIAGNOSIS_CAN_H
#include <stdbool.h>
#include "diagnosis_contract.h"
bool DiagnosisCan_Configure(uint32_t channel, uint32_t bitrate, uint32_t mode);
uint32_t DiagnosisCan_Send(const DiagnosisRequest *request, uint32_t now_ms);
bool DiagnosisCan_Poll(DiagnosisEvent *event, uint32_t now_ms);
void DiagnosisCan_Snapshot(uint32_t channel, DiagnosisBus *bus);
#endif
