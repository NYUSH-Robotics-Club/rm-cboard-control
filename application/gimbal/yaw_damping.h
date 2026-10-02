/* 云台命令上下文独占的近端制动状态；不访问电机或硬件。 */
#ifndef YAW_DAMPING_H
#define YAW_DAMPING_H
#include "config_types.h"
typedef struct {
  bool holding;
  bool target_valid;
  float previous_target_ticks;
  float approach_direction;
} YawDampingState;
bool YawDamping_ConfigValid(const YawControlConfig *cfg);
void YawDamping_Reset(YawDampingState *state);
/* error_deg为连续目标误差；target_ticks仅用于识别目标主动反向。
 * 调用方须验证反馈并在停机、模式切换、速度环旁路时Reset。 */
float YawDamping_Update(YawDampingState *state, const YawControlConfig *cfg,
                        float error_deg, float actual_rpm, float target_ticks);
#endif
