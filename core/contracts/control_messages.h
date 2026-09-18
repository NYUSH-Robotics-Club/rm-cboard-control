/*
 * 定义应用模块之间传递的底盘、云台和发射命令。
 * 这些结构不包含控制器内部状态或硬件句柄。
 */
#ifndef CONTROL_MESSAGES_H
#define CONTROL_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>

/* Application-level contracts. These types contain no controller or HAL state. */
typedef struct {
    float vx;
    float vy;
    float wz;
    bool enabled;
} ChassisCmd;

typedef struct {
    bool friction_enabled;
    bool feed_enabled;
} ShootCmd;

enum {
    GIMBAL_TRACE_PRESENT = 1U,
    GIMBAL_TRACE_RC_SEEN = 2U,
    GIMBAL_TRACE_RC_ONLINE = 4U,
    GIMBAL_TRACE_OUTPUTS_ARMED = 8U,
    GIMBAL_TRACE_VISION_REQUESTED = 16U,
    GIMBAL_TRACE_SPIN_REQUESTED = 32U
};

/* Cmd随命令复制的来源；只用于观测，不参与控制。序号自然回绕。
 * rc_sequence计应用收到的RC消息，rc_dispatch_ms是消息派发时刻，不是UART到达时刻。
 * ch0为解码后、死区/取反/过滤前的杆量（合法范围±660）；无RC时由flags标明无效。
 */
typedef struct {
    uint32_t flags;
    uint32_t rc_sequence;
    uint32_t rc_dispatch_ms;
    uint32_t route_sequence;
    uint32_t route_ms;
    int32_t rc_ch0;
} GimbalCommandTrace;

typedef struct {
    bool enabled;
    float pitch_rate;
    float yaw_rate;
    float yaw_rate_memo;
    float yaw_target_memo;
    bool vision_valid;
    float vision_yaw_err_rad;
    float vision_pitch_err_rad;
    uint32_t vision_ts_ms;
    GimbalCommandTrace trace; /* 零初始化表示启动等非遥控路由来源，没有伪造输入。 */
} GimbalCmd;

#endif
