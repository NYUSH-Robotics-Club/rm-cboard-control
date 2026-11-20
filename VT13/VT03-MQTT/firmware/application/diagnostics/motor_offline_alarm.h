/*
 * 按车型配置轮流报告电机反馈失联；只提示，不改变电机控制策略。
 * 所有时间为毫秒，状态由主循环持有，不在中断中调用。
 */
#ifndef MOTOR_OFFLINE_ALARM_H
#define MOTOR_OFFLINE_ALARM_H
#include <stdint.h>
#include "config_types.h"

typedef struct {
    uint16_t offline_mask;  /* bit为软件motor_id，含从未反馈、服务不可用和年龄>100ms。 */
    uint16_t unmapped_mask; /* 在线状态不受影响；配置缺少合法报码的电机位。 */
    uint8_t active_motor_id; /* 当前组的软件ID，0xFF表示无活动组。 */
    uint8_t bus_number;      /* CAN1=1，CAN2=2；0表示配置不支持。 */
    uint8_t hardware_id;     /* 当前红灯次数，0表示配置缺失，用常红代替猜测。 */
    uint8_t blue_separator;  /* 当前在蓝灯分隔阶段。 */
    uint8_t config_invalid;  /* 无配置/超过16台/软件ID非法或重复。 */
} MotorOfflineAlarmDiagnostics;

/* 保存只读车型引用并清状态；配置必须在整个运行期有效，错误用常红提示。 */
void MotorOfflineAlarm_Init(const RobotConfig_t *robot);
/* 在启动校准完成后每轮调用；未显式Init则取当前车型。无阻塞、无延时。
 * 红灯100ms亮/100ms灭，每组至少1s，之后蓝灯400ms；每组蜂鸣总线号次数。
 * 全部在线立即静音亮绿；循环停顿>=100ms时用蓝灯结束残组，避免补发密集脉冲。
 */
void MotorOfflineAlarm_Task(uint32_t now_ms);
/* 返回模块持有的只读诊断；主循环或停止目标后读取。 */
const MotorOfflineAlarmDiagnostics *MotorOfflineAlarm_GetDiagnostics(void);
#endif
