#ifndef DIAGNOSIS_H
#define DIAGNOSIS_H
#include "diagnosis_contract.h"
extern volatile DiagnosisShared g_diagnosis;
void Diagnosis_Init(void);
void Diagnosis_Step(uint32_t now_ms);
#endif
