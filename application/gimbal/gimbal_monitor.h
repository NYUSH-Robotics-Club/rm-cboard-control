/*
 * 定义云台回调结束时发布的只读监控快照；电脑通过SWD读取，不在回调中打印。
 * 这是控制回调采样，不是逐帧CAN记录；命令被服务接受也不等于已在线路发送。
 */
#ifndef GIMBAL_MONITOR_H
#define GIMBAL_MONITOR_H
#include <stdint.h>

enum {
    GIMBAL_MONITOR_PRESENT = 1U,
    GIMBAL_MONITOR_FEEDBACK_SEEN = 2U,
    GIMBAL_MONITOR_FEEDBACK_FRESH = 4U,
    GIMBAL_MONITOR_CONTROL_ACTIVE = 8U,
    GIMBAL_MONITOR_POSITION_ACTIVE = 16U,
    GIMBAL_MONITOR_SPEED_IMU = 32U
};

/* 所有字段占4字节，供主机按固定v1布局解码；缺轴flags为0，目标无效时不得使用。 */
typedef struct {
    uint32_t motor_id;
    uint32_t flags;
    uint32_t feedback_ms;
    uint32_t command_unit; /* MotorCommandUnit；未知单位不推断电流。 */
    int32_t command_status; /* RobotStatus；失败时命令只是未成功的请求。 */
    int32_t command_raw; /* 包含前馈/重力且经应用限幅后的服务请求，非PID.output。 */
    int32_t current_actual_raw; /* 电调实际电流反馈刻度，非安培。 */
    float position_actual_ticks; /* yaw有效时连续角；pitch单圈绝对编码。 */
    float position_target_ticks; /* 与actual同坐标；速度调试时仅为备忘目标。 */
    float speed_actual_rpm; /* 电机原始转子RPM。 */
    float speed_target_rpm; /* 本次速度内环目标；仅CONTROL_ACTIVE时有效。 */
    float speed_loop_actual_rpm; /* 内环实际采用的反馈；spin可能为IMU。 */
    float speed_loop_dt_s;
    uint32_t encoder_raw; /* 单圈原生编码0..8191，便于检查跨零。 */
} GimbalMonitorAxis;

typedef struct {
    uint32_t sequence; /* 单写者发布：奇数在写，读取前后必须为同一偶数。 */
    uint32_t magic; /* 0x474D4F4E，版本或大小不符须停止读取。 */
    uint32_t version;
    uint32_t size_bytes;
    uint32_t callback_count; /* 有效云台命令回调数，uint32自然回绕。 */
    uint32_t tick_ms;
    uint32_t callback_dt_ms;
    uint32_t enabled;
    uint32_t startup_ready;
    GimbalMonitorAxis yaw;
    GimbalMonitorAxis pitch;
} GimbalMonitorSnapshot;

/* 由云台消息派发上下文独占写入；调试器只读。禁止ISR或另一个任务修改。 */
extern volatile GimbalMonitorSnapshot g_gimbal_monitor;
#endif
