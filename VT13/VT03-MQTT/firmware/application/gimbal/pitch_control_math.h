#ifndef PITCH_CONTROL_MATH_H
#define PITCH_CONTROL_MATH_H
#include "config_types.h"
bool PitchControl_ConfigValid(const PitchControlConfig *cfg);
/* 经验负载电压指令：实测范围外钳位，不外推，不再次乘motor direction。 */
float PitchControl_Load(const PitchControlConfig *cfg, float position);
#endif
