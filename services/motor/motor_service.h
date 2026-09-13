/*
 * 声明应用层使用的统一电机服务。
 * 调用方不需要了解厂商 CAN 帧，但必须检查返回状态。
 */
#ifndef MOTOR_SERVICE_H
#define MOTOR_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include "config_types.h"
#include "motor_adapter.h"
#include "robot_status.h"

/* Safe to call repeatedly. Binds the motor flush to the message-cycle hook. */
RobotStatus MotorService_Init(void);

const MotorConfig_t *MotorService_GetConfig(uint8_t motor_id);
uint8_t MotorService_FindByRole(MotorRole_e role,
                                uint8_t *motor_ids,
                                uint8_t max_count);
RobotStatus MotorService_GetSnapshot(uint8_t motor_id,
                                     MotorSnapshot *snapshot);

/*
 * Compatibility path for existing application-owned control loops. DISABLED
 * always sends zero. SPEED/POSITION modes reject direct-current commands so a
 * tuning mistake cannot silently bypass the selected loop.
 */
RobotStatus MotorService_CommandCurrent(uint8_t motor_id, int16_t current);

/*
 * Execute the control mode selected in MotorConfig_t:
 *   APPLICATION: sends application_current;
 *   OPEN_LOOP_CURRENT: sends setpoint as current;
 *   SPEED/POSITION/CASCADE: adapter computes current from setpoint;
 *   DISABLED: sends zero.
 */
RobotStatus MotorService_CommandConfigured(uint8_t motor_id,
                                           float setpoint,
                                           int16_t application_current);

/* Runtime tuning override. Clear it to return to the const robot config. */
RobotStatus MotorService_SetControlMode(uint8_t motor_id,
                                        MotorControlMode_e mode);
RobotStatus MotorService_ClearControlModeOverride(uint8_t motor_id);
MotorControlMode_e MotorService_GetControlMode(uint8_t motor_id);

/* 原始命令的物理种类，不提供安培/伏特换算；未知协议或编号返回UNKNOWN。 */
typedef enum {
    MOTOR_COMMAND_UNIT_UNKNOWN = 0,
    MOTOR_COMMAND_UNIT_CURRENT_COUNTS = 1,
    MOTOR_COMMAND_UNIT_VOLTAGE_COUNTS = 2
} MotorCommandUnit;
MotorCommandUnit MotorService_GetCommandUnit(uint8_t motor_id);

bool MotorService_IsVendorImplemented(MotorVendor_e vendor);
const char *MotorService_VendorName(MotorVendor_e vendor);
void MotorService_Flush(void);

#endif
