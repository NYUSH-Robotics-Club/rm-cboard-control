/*
 * Receives decoded input messages, asks CommandRouter for one command set, and
 * publishes it. Mode policy stays in command_router.c; no motor is driven here.
 */
#include "cmd_controller.h"
#include "command_router.h"
#include "logger.h"
#include "message_center.h"
#include "bsp_can.h"
#include "motor_service.h"
#include "robot_config.h"
#include "bsp_time.h"
#include <math.h>
#include <string.h>

static CommandRouterInput s_input;
static CommandRouterOutput s_output;
static CommandRouter s_router;
static bool s_initialized = false;
static bool s_remote_updated;
static bool s_remote_seen;
static uint32_t s_last_remote_ms;
static bool s_neutral_seen;
static uint32_t s_neutral_since_ms;
/* 派发上下文独占；计数是已交付消息，不是UART接收帧数。 */
static uint32_t s_rc_sequence, s_rc_dispatch_ms, s_route_sequence;

_Static_assert(sizeof(GimbalCmd) <= MC_MAX_PAYLOAD, "gimbal command exceeds message slot");

#define REMOTE_LOSS_TIMEOUT_MS (200U)

static void update_yaw_heading(void) {
    const RobotConfig_t *robot = RobotConfig_Get();
    const ChassisFollowConfig *follow = robot ? robot->chassis_follow : NULL;
    s_input.encoder_follow = follow != NULL;
    s_input.yaw_heading_valid = false;
    if (!follow || follow->yaw_forward_ticks >= 8192U ||
        (follow->yaw_ccw_sign != 1 && follow->yaw_ccw_sign != -1)) return;

    uint8_t ids[2];
    if (MotorService_FindByRole(MOTOR_ROLE_GIMBAL_YAW, ids, 2U) != 1U) return;
    const MotorConfig_t *motor = MotorService_GetConfig(ids[0]);
    /* 此安装标定只支持已知的DJI GM6020单圈刻度，其他协议单位不能套用。 */
    if (!motor || motor->vendor != MOTOR_VENDOR_DJI || motor->type != MOTOR_TYPE_GM6020)
        return;
    MotorSnapshot snapshot;
    if (MotorService_GetSnapshot(ids[0], &snapshot) != ROBOT_STATUS_OK ||
        !snapshot.initialized || !snapshot.feedback_valid ||
        !isfinite(snapshot.position) || snapshot.position < 0.0f ||
        snapshot.position >= 8192.0f) return;

    float ticks = snapshot.position - (float)follow->yaw_forward_ticks;
    if (ticks > 4096.0f) ticks -= 8192.0f;
    if (ticks < -4096.0f) ticks += 8192.0f;
    s_input.yaw_relative_deg = ticks * (360.0f / 8192.0f) * follow->yaw_ccw_sign;
    s_input.yaw_feedback_ms = snapshot.feedback_timestamp_ms;
    s_input.yaw_heading_valid = true;
}

static void on_rc_update(const MsgEvent *event, void *user_data) {
    (void)user_data;
    if (event->size == sizeof(s_input.remote)) {
        memcpy(&s_input.remote, event->data, sizeof(s_input.remote));
        ++s_rc_sequence;
        s_rc_dispatch_ms = BspTime_NowMs();
        s_remote_updated = true;
    }
}

static void on_imu_update(const MsgEvent *event, void *user_data) {
    (void)user_data;
    if (event->size == sizeof(s_input.sensor)) {
        memcpy(&s_input.sensor, event->data, sizeof(s_input.sensor));
    }
}

static void on_vision_update(const MsgEvent *event, void *user_data) {
    (void)user_data;
    if (event->size == sizeof(s_input.vision)) {
        memcpy(&s_input.vision, event->data, sizeof(s_input.vision));
        s_input.vision_updated = true;
    }
}

void CmdController_Init(void) {
    if (s_initialized) {
        return;
    }

    /* 连续输入和命令只需最新值，独立于 CAN 逐条事件队列；容量不足时不启动控制。 */
    const MsgTopic inputs[] = {TOPIC_RC_UPDATE, TOPIC_IMU_UPDATE, TOPIC_VISION_TARGET};
    const MsgTopic commands[] = {TOPIC_CHASSIS_CMD, TOPIC_SHOOT_CMD, TOPIC_GIMBAL_CMD};
    for (size_t i = 0U; i < sizeof(inputs) / sizeof(inputs[0]); ++i) {
        if (MsgCenter_UseLatest(inputs[i], MC_LATEST_STATE) != 0) return;
    }
    for (size_t i = 0U; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        if (MsgCenter_UseLatest(commands[i], MC_LATEST_CONTROL) != 0) return;
    }

    memset(&s_input, 0, sizeof(s_input));
    memset(&s_output, 0, sizeof(s_output));
    s_remote_updated = false;
    s_remote_seen = false;
    s_last_remote_ms = 0U;
    s_rc_sequence = s_rc_dispatch_ms = s_route_sequence = 0U;
    CommandRouter_Init(&s_router);

    (void)MsgCenter_Subscribe(TOPIC_RC_UPDATE, on_rc_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);
    (void)MsgCenter_Subscribe(TOPIC_VISION_TARGET, on_vision_update, NULL);
    s_initialized = true;
}

void CmdController_Task(uint32_t current_tick) {
    if (!s_initialized) {
        return;
    }
    BspCan_Service(current_tick);

    if (s_remote_updated) {
        s_remote_updated = false;
        s_remote_seen = true;
        s_last_remote_ms = current_tick;
    }
    s_input.remote_online = s_remote_seen &&
        (uint32_t)(current_tick - s_last_remote_ms) <= REMOTE_LOSS_TIMEOUT_MS;

    bool remote_online = s_input.remote_online;
    if (!BspCan_OutputsArmed()) {
        bool neutral = remote_online && BspCan_RecoveryReady(current_tick) &&
            switch_is_down(s_input.remote.rc.s[0]) && switch_is_down(s_input.remote.rc.s[1]);
        for (unsigned i = 0U; i < 5U; ++i) {
            if (s_input.remote.rc.ch[i] < -3 || s_input.remote.rc.ch[i] > 3) neutral = false;
        }
        if (!neutral) s_neutral_seen = false;
        else if (!s_neutral_seen) {
            s_neutral_seen = true;
            s_neutral_since_ms = current_tick;
        } else if ((uint32_t)(current_tick - s_neutral_since_ms) >= 500U) {
            (void)BspCan_TryArm(current_tick);
            s_neutral_seen = false;
        }
        /* Publish one final disabled command set even on the arming cycle. */
        s_input.remote_online = false;
    } else {
        s_neutral_seen = false;
    }

    update_yaw_heading();
    RobotStatus route_status = CommandRouter_Route(&s_router,
                                                    &s_input,
                                                    current_tick,
                                                    &s_output);
    s_input.remote_online = remote_online;
    if (route_status != ROBOT_STATUS_OK &&
        route_status != ROBOT_STATUS_NOT_READY) {
        return;
    }
    s_input.vision_updated = false;

    /* 把本次路由实际使用的输入随结果发送，避免与下一条RC消息错配。 */
    s_output.gimbal.trace = (GimbalCommandTrace){
        .flags = GIMBAL_TRACE_PRESENT |
            (s_remote_seen ? GIMBAL_TRACE_RC_SEEN : 0U) |
            (remote_online ? GIMBAL_TRACE_RC_ONLINE : 0U) |
            (BspCan_OutputsArmed() ? GIMBAL_TRACE_OUTPUTS_ARMED : 0U) |
            (s_output.gimbal.vision_valid ? GIMBAL_TRACE_VISION_REQUESTED : 0U) |
            (s_output.gimbal.yaw_rate_memo > 0.5f ? GIMBAL_TRACE_SPIN_REQUESTED : 0U),
        .rc_sequence = s_rc_sequence,
        .rc_dispatch_ms = s_rc_dispatch_ms,
        .route_sequence = ++s_route_sequence,
        .route_ms = current_tick,
        .rc_ch0 = s_input.remote.rc.ch[0],
    };

    float yaw_error_deg =
        s_output.spin_hold_yaw_deg - s_input.sensor.yaw_total_angle;
    LOG_CSV(LOG_TAG_CMD, "%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f",
            (unsigned int)(s_output.spin_mode ? 1U : 0U),
            (unsigned int)((uint8_t)s_input.remote.rc.s[0]),
            (unsigned int)((uint8_t)s_input.remote.rc.s[1]),
            s_input.sensor.c_yaw,
            s_input.sensor.yaw_total_angle,
            s_output.spin_hold_yaw_deg,
            yaw_error_deg,
            s_output.gimbal.yaw_rate,
            s_output.chassis.vx,
            s_output.chassis.vy,
            s_output.chassis.wz);

    (void)MsgCenter_Publish(TOPIC_CHASSIS_CMD,
                            &s_output.chassis,
                            sizeof(s_output.chassis));
    (void)MsgCenter_Publish(TOPIC_SHOOT_CMD,
                            &s_output.shooter,
                            sizeof(s_output.shooter));
    (void)MsgCenter_Publish(TOPIC_GIMBAL_CMD,
                            &s_output.gimbal,
                            sizeof(s_output.gimbal));
}
