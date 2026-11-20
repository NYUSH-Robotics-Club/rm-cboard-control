/* 非 DJI 适配器共用的 PID 开关，避免每个厂商重复控制算法。 */
#ifndef VENDOR_CONTROL_H
#define VENDOR_CONTROL_H

#include <stdint.h>
#include "motor_adapter.h"
#include "pid.h"

typedef struct {
    const MotorConfig_t *config;
    PID_Controller outer;
    PID_Controller inner;
    MotorSnapshot snapshot;
} VendorControl;

void VendorControl_Init(VendorControl *control, const MotorConfig_t *config);
RobotStatus VendorControl_Compute(VendorControl *control,
                                  MotorControlMode_e mode,
                                  float setpoint,
                                  int16_t application_current,
                                  int16_t command_limit,
                                  int16_t *output);
void VendorControl_Reset(VendorControl *control);

#endif
