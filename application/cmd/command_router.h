/*
 * Converts decoded operator, sensor, and vision inputs into the three robot
 * command messages. It owns mode priority but does not publish or drive motors.
 */
#ifndef COMMAND_ROUTER_H
#define COMMAND_ROUTER_H

#include <stdbool.h>
#include <stdint.h>
#include "control_messages.h"
#include "remote_messages.h"
#include "robot_status.h"
#include "sensor_messages.h"
#include "vision_messages.h"

typedef struct {
    RemoteControlMessage remote;
    SensorData sensor;
    VisionTargetMessage vision;
    bool vision_updated;
    bool remote_online;
    bool encoder_follow;          /* 该车型中档使用yaw编码器坐标。 */
    bool yaw_heading_valid;       /* 缺失或非法反馈时禁止中档底盘运动。 */
    float yaw_relative_deg;       /* 相对底盘正前方，俯视逆时针为正。 */
    uint32_t yaw_feedback_ms;      /* 实际电机反馈时间，不以路由时间代替。 */
} CommandRouterInput;

typedef struct {
    ChassisCmd chassis;
    ShootCmd shooter;
    GimbalCmd gimbal;
    bool spin_mode;
    bool gimbal_follow_mode;
    float spin_hold_yaw_deg;
} CommandRouterOutput;

typedef struct {
    bool spin_mode;
    bool gimbal_follow_mode;
    float spin_hold_yaw_deg;
    float previous_yaw_input;
    uint32_t previous_route_ms;
    bool route_time_valid;
    bool shooter_down_seen; /* Startup or link loss requires a fresh down position. */
    GimbalCmd gimbal_memory;
    float dial_wz;/* 平滑后的底盘旋转输入，归一化单位，范围约为-1到1。 */
    bool dial_active;/* 拨轮是否越过进入阈值；掉线和spin模式时清零。 */
} CommandRouter;

void CommandRouter_Init(CommandRouter *router);

/*
 * Build one command set. now_ms controls feedback/vision expiry and spin dt.
 * Outputs are normalized except for documented angle fields in GimbalCmd.
 * Encoder follow disables the chassis when yaw feedback is invalid or >20 ms old.
 */
RobotStatus CommandRouter_Route(CommandRouter *router,
                                const CommandRouterInput *input,
                                uint32_t now_ms,
                                CommandRouterOutput *output);

#endif
