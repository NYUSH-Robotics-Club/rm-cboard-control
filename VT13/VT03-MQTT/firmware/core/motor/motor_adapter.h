/*
 * 定义各电机厂商都要实现的统一操作。
 * 厂商协议只存在于适配器，应用层只处理统一命令和反馈。
 */
#ifndef MOTOR_ADAPTER_H
#define MOTOR_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>
#include "config_types.h"
#include "can_messages.h"
#include "robot_status.h"
#include "motor_trace.h"

typedef struct {
    uint8_t motor_id;
    bool initialized;
    bool feedback_valid;
    float position;
    float speed;
    float torque_or_current;
    uint32_t feedback_timestamp_ms;
} MotorSnapshot;

typedef struct MotorAdapterOps {
    MotorVendor_e vendor;
    const char *name;
    bool implemented;
    RobotStatus (*init)(const RobotConfig_t *robot);
    RobotStatus (*validate)(const MotorConfig_t *config);
    RobotStatus (*command_current)(uint8_t motor_id, int16_t current);
    RobotStatus (*stop)(uint8_t motor_id);
    RobotStatus (*compute_current)(uint8_t motor_id,
                                   MotorControlMode_e mode,
                                   float setpoint,
                                   int16_t application_current,
                                   int16_t *output_current);
    RobotStatus (*snapshot)(uint8_t motor_id, MotorSnapshot *snapshot);
    void (*on_can_frame)(const CanRxFrame *frame);
    void (*reset_control)(uint8_t motor_id);
    void (*flush)(void);
    void (*read_trace)(MotorTraceBatch *batch); /* Optional, task-context diagnostic drain. */
} MotorAdapterOps;

const MotorAdapterOps *MotorAdapter_Get(MotorVendor_e vendor);
const MotorAdapterOps *DjiMotorAdapter_Get(void);
const MotorAdapterOps *DmMotorAdapter_Get(void);
const MotorAdapterOps *BenmoMotorAdapter_Get(void);
const MotorAdapterOps *LkMotorAdapter_Get(void);

#endif
