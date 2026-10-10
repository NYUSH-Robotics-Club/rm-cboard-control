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
#if defined(ROBOT_TYPE_sentry_swerve) || defined(ROBOT_TYPE_infantry_standard)
#include "vision_comm.h"
#endif
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
#if defined(ROBOT_TYPE_infantry_standard)
static bool s_autonomous_route_active;
#endif
/* 派发上下文独占；计数是已交付消息，不是UART接收帧数。 */
static uint32_t s_rc_sequence, s_rc_dispatch_ms, s_route_sequence;

_Static_assert(sizeof(GimbalCmd) <= MC_MAX_PAYLOAD, "gimbal command exceeds message slot");

#define REMOTE_LOSS_TIMEOUT_MS (200U)

static float normalize_angle_180(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static void update_yaw_heading(void) {
    const RobotConfig_t *robot = RobotConfig_Get();
    const ChassisFollowConfig *follow = robot ? robot->chassis_follow : NULL;
    s_input.encoder_follow = follow != NULL;
    s_input.yaw_heading_valid = false;
    s_input.yaw_speed_valid = false;
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
    s_input.yaw_speed_rpm = snapshot.speed;
    s_input.yaw_speed_ccw_sign = follow->yaw_ccw_sign;
    s_input.yaw_speed_valid = true;
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
#if defined(ROBOT_TYPE_infantry_standard)
    s_autonomous_route_active = false;
#endif
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
    /* Optional modules may submit CAN after the cycle-start timestamp.
     * Use the current HAL/BSP clock so mailbox ages cannot underflow. */
    BspCan_Service(BspTime_NowMs());

    if (s_remote_updated) {
        s_remote_updated = false;
        s_remote_seen = true;
        s_last_remote_ms = current_tick;
    }
    s_input.remote_online = s_remote_seen &&
        (uint32_t)(current_tick - s_last_remote_ms) <= REMOTE_LOSS_TIMEOUT_MS;

    bool remote_online = s_input.remote_online;
#if defined(ROBOT_TYPE_sentry_swerve) || defined(ROBOT_TYPE_infantry_standard)
    SentryBridgeCommand sentry_command;
    bool sentry_online = VisionComm_GetSentryCommand(&sentry_command);
#else
    bool sentry_online = false;
#endif
#if defined(ROBOT_TYPE_infantry_standard)
    const uint8_t odin_localization_scan = 0x20U;
    const uint8_t odin_autonomy = 0x40U;
    bool autonomous = sentry_online &&
        (sentry_command.control_flags & odin_autonomy) != 0U;
    bool nav_autonomous = autonomous &&
        (sentry_command.control_flags & odin_localization_scan) == 0U;
#endif
    /* Latch the pre-cycle arm state: recovery must still publish one disabled
     * cycle even if TryArm succeeds below. */
    bool outputs_armed_for_cycle = BspCan_OutputsArmed();
    if (!outputs_armed_for_cycle) {
        bool neutral = remote_online && BspCan_RecoveryReady(current_tick) &&
            switch_is_down(s_input.remote.rc.s[0]) && switch_is_down(s_input.remote.rc.s[1]);
        if (remote_online) {
            for (unsigned i = 0U; i < 5U; ++i) {
                if (s_input.remote.rc.ch[i] < -3 || s_input.remote.rc.ch[i] > 3) neutral = false;
            }
        }
#if defined(ROBOT_TYPE_infantry_standard)
        /* Odin can arm after the same stable CAN interval without an RC.
         * Require a fresh dedicated bridge keepalive and zero chassis speed. */
        if (!remote_online && autonomous && BspCan_RecoveryReady(current_tick) &&
            isfinite(sentry_command.vx) && isfinite(sentry_command.vy) &&
            isfinite(sentry_command.wz) &&
            fabsf(sentry_command.vx) < 0.001f &&
            fabsf(sentry_command.vy) < 0.001f &&
            fabsf(sentry_command.wz) < 0.001f) neutral = true;
#endif
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
        sentry_online = false;
    } else {
        s_neutral_seen = false;
    }

    update_yaw_heading();
    /* The bridge command is a chassis-level override. It shares the existing
     * command topic, so swerve kinematics and CAN output remain unchanged. */
#if defined(ROBOT_TYPE_sentry_swerve)
    s_input.remote_online = outputs_armed_for_cycle &&
                            (remote_online || sentry_online);
#else
    CommandRouterInput route_input = s_input;
    if (!remote_online && autonomous) {
        if (!s_autonomous_route_active) CommandRouter_Init(&s_router);
        s_autonomous_route_active = true;
        /* Give the router neutral operator controls. Stale RC switches must
         * not activate spin or the shooter when autonomous control takes over. */
        memset(&route_input.remote, 0, sizeof(route_input.remote));
        route_input.remote.rc.s[0] = RC_SW_DOWN;
        route_input.remote.rc.s[1] = RC_SW_DOWN;
    } else s_autonomous_route_active = false;
    route_input.remote_online = outputs_armed_for_cycle &&
                                (remote_online || autonomous);
#endif
#if defined(ROBOT_TYPE_infantry_standard)
    RobotStatus route_status = CommandRouter_Route(&s_router,
                                                    &route_input,
                                                    current_tick,
                                                    &s_output);
#else
    RobotStatus route_status = CommandRouter_Route(&s_router,
                                                    &s_input,
                                                    current_tick,
                                                    &s_output);
#endif
    s_input.remote_online = remote_online;
    if (route_status != ROBOT_STATUS_OK &&
        route_status != ROBOT_STATUS_NOT_READY) {
        return;
    }
    s_input.vision_updated = false;
#if defined(ROBOT_TYPE_infantry_standard)
    if (!remote_online) {
        s_output.shooter = (ShootCmd){0};
        s_router.shooter_down_seen = false;
    }
#endif

#if defined(ROBOT_TYPE_sentry_swerve)
    if (sentry_online) {
        s_output.chassis.vx = sentry_command.vx;
        s_output.chassis.vy = sentry_command.vy;
        s_output.chassis.wz = sentry_command.wz;
        s_output.chassis.enabled = true;
    }
    VisionComm_SetSentryTelemetry(s_output.chassis.vx, s_output.chassis.vy,
                                  s_output.chassis.wz,
                                  0.0f, 0.0f, 0.0f);
#elif defined(ROBOT_TYPE_infantry_standard)
    /* RC chassis axes take priority while the transmitter is online. Odin
     * may drive with a fresh dedicated keepalive when it is offline. */
    const RemoteChannels *rc = &s_input.remote.rc;
    bool manual_chassis =
        rc->ch[2] < -3 || rc->ch[2] > 3 ||
        rc->ch[3] < -3 || rc->ch[3] > 3 ||
        rc->ch[4] < -15 || rc->ch[4] > 15;
    if (sentry_online && outputs_armed_for_cycle &&
        (remote_online || nav_autonomous) &&
        (!remote_online || !manual_chassis)) {
        float chassis_vx = 0.0f, chassis_vy = 0.0f;
        bool command_valid = isfinite(sentry_command.vx) &&
            isfinite(sentry_command.vy) && isfinite(sentry_command.wz) &&
            fabsf(sentry_command.vx) <= 1.0f &&
            fabsf(sentry_command.vy) <= 1.0f &&
            fabsf(sentry_command.wz) <= 1.0f &&
            sentry_command.vx * sentry_command.vx +
                sentry_command.vy * sentry_command.vy <= 1.0001f;
        /* Odin/Nav2 uses the gimbal-forward virtual base frame. Match the RC
         * middle-position conversion before the normalized chassis command
         * reaches the omni wheel strategy. Stale yaw must stop autonomous
         * output even when the RC switch is in spin mode. */
        if (command_valid && CommandRouter_GimbalToChassis(
                &s_input, current_tick, sentry_command.vx, sentry_command.vy,
                &chassis_vx, &chassis_vy)) {
            s_output.chassis.vx = chassis_vx;
            s_output.chassis.vy = chassis_vy;
            s_output.chassis.wz = sentry_command.wz;
            s_output.chassis.enabled = true;
        } else {
            s_output.chassis = (ChassisCmd){0};
        }
    }
#endif

#if defined(ROBOT_TYPE_infantry_standard)
    /* SX robot-control scan is only a temporary localization aid. RC gimbal
     * sticks and vision keep priority; spin/exit-brake modes keep their hold.
     * The bridge's 250 ms SX expiry stops a stale scan automatically. */
    const uint8_t scan_valid = 0x01U;
    const uint8_t stop_scan = 0x02U;
    const uint8_t scan_enabled = 0x04U;
    if (sentry_online && (remote_online || autonomous) && outputs_armed_for_cycle &&
        (sentry_command.control_flags &
            (scan_valid | scan_enabled | odin_localization_scan)) ==
            (scan_valid | scan_enabled | odin_localization_scan) &&
        (sentry_command.control_flags & stop_scan) == 0U &&
        !s_output.spin_mode && !s_output.gimbal.vision_valid &&
        s_output.gimbal.yaw_rate_memo < 0.5f &&
        (!remote_online || (s_input.remote.rc.ch[0] >= -3 && s_input.remote.rc.ch[0] <= 3)) &&
        (!remote_online || (s_input.remote.rc.ch[1] >= -3 && s_input.remote.rc.ch[1] <= 3)) &&
        isfinite(sentry_command.scan_yaw_rate_deg_s) &&
        fabsf(sentry_command.scan_yaw_rate_deg_s) <= 60.0f) {
        uint8_t ids[1];
        if (MotorService_FindByRole(MOTOR_ROLE_GIMBAL_YAW, ids, 1U) == 1U) {
            const MotorConfig_t *yaw = MotorService_GetConfig(ids[0]);
            const YawControlConfig *cfg = yaw ? yaw->yaw_control : NULL;
            if (cfg && isfinite(cfg->manual_rate_deg_s) &&
                isfinite(cfg->manual_stick_gain) &&
                cfg->manual_rate_deg_s > 0.0f && cfg->manual_stick_gain > 0.0f) {
                float full_rate = cfg->manual_rate_deg_s * cfg->manual_stick_gain;
                if (isfinite(full_rate) && full_rate >= 60.0f) {
                    s_output.gimbal.yaw_rate =
                        sentry_command.scan_yaw_rate_deg_s / full_rate;
                    /* Motor RPM is degrees per second divided by six. Keep
                     * the cap tied to the accepted localization request. */
                    s_output.gimbal.yaw_speed_cap_rpm =
                        fabsf(sentry_command.scan_yaw_rate_deg_s) / 6.0f;
                }
            }
        }
    }
#endif

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
    float yaw_world = normalize_angle_180(s_input.sensor.yaw_total_angle);
    float chassis_world = normalize_angle_180(s_input.sensor.c_yaw);
    float yaw_to_chassis = normalize_angle_180(yaw_world - chassis_world);
    LOG_CSV(LOG_TAG_CMD, "%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f",
            (unsigned int)(s_output.spin_mode ? 1U : 0U),
            (unsigned int)((uint8_t)s_input.remote.rc.s[0]),
            (unsigned int)((uint8_t)s_input.remote.rc.s[1]),
            s_input.sensor.c_yaw,
            s_input.sensor.yaw_total_angle,
            s_output.spin_hold_yaw_deg,
            yaw_error_deg,
            yaw_world,
            chassis_world,
            yaw_to_chassis,
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
