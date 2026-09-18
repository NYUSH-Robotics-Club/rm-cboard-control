/*
 * Holds the robot's manual, spin, follow, and vision command priority rules.
 * Transport callbacks and message publication stay in cmd_controller.c.
 */
#include "command_router.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define VISION_CMD_TIMEOUT_MS (20U)
#define JOYSTICK_DEADBAND (3)
#define MAX_ROUTE_DT_S (0.050f)
#define DEG_TO_RAD (0.01745329251994329577f)
#define FOLLOW_FEEDBACK_TIMEOUT_MS (20U)

#define SPIN_WZ_NORM (0.33f)
#define SPIN_TRANSLATE_LIMIT_NORM (1.00f)
#define SPIN_GIMBAL_YAW_ADJ_DEG_PER_S (120.0f)

static float normalize_angle_180(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void gimbal_to_chassis_frame(float vx_g,
                                    float vy_g,
                                    float offset_angle_deg,
                                    float *vx_c,
                                    float *vy_c) {
    const float angle_rad = offset_angle_deg * DEG_TO_RAD;
    const float cosine = cosf(angle_rad);
    const float sine = sinf(angle_rad);
    *vx_c = cosine * vx_g - sine * vy_g;
    *vy_c = sine * vx_g + cosine * vy_g;
}

static int16_t apply_deadband(int16_t value) {
    return (value > -JOYSTICK_DEADBAND && value < JOYSTICK_DEADBAND)
               ? 0
               : value;
}

static void route_chassis(const CommandRouterInput *input,
                          uint32_t now_ms,
                          bool spin_mode,
                          bool gimbal_follow_mode,
                          ChassisCmd *command) {
    const RemoteControlMessage *remote = &input->remote;
    const SensorData *sensor = &input->sensor;
    int16_t vx_raw = apply_deadband(remote->rc.ch[3]);
    int16_t vy_raw = apply_deadband(remote->rc.ch[2]);
    int16_t wz_raw = apply_deadband(remote->rc.ch[4]);
    const float max_input = (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    float vx = -(float)vx_raw / max_input;
    float vy = (float)vy_raw / max_input;
    float wz = (float)wz_raw / max_input;

    if (gimbal_follow_mode && input->encoder_follow) {
        /* 用实际云台相对角旋转平移向量，+x前、+y左；不再额外交换轴。
         * 方位失效就禁用整条底盘命令，不能退回另一坐标系继续运动。
         */
        if (!input->yaw_heading_valid || !isfinite(input->yaw_relative_deg) ||
            (uint32_t)(now_ms - input->yaw_feedback_ms) > FOLLOW_FEEDBACK_TIMEOUT_MS) {
            return;
        }
        gimbal_to_chassis_frame(vx, vy, input->yaw_relative_deg,
                               &command->vx, &command->vy);
        command->wz = wz;
        command->enabled = (vx_raw != 0 || vy_raw != 0 || wz_raw != 0);
        return;
    }

    if (spin_mode || gimbal_follow_mode) {
        float gimbal_yaw = normalize_angle_180(sensor->yaw_total_angle);
        float chassis_yaw = normalize_angle_180(sensor->c_yaw);
        float offset = normalize_angle_180(chassis_yaw - gimbal_yaw);
        float chassis_vx = 0.0f;
        float chassis_vy = 0.0f;
        gimbal_to_chassis_frame(vx, vy, offset, &chassis_vx, &chassis_vy);

        if (spin_mode) {
            float magnitude = sqrtf(chassis_vx * chassis_vx +
                                    chassis_vy * chassis_vy);
            if (magnitude > SPIN_TRANSLATE_LIMIT_NORM) {
                float scale = SPIN_TRANSLATE_LIMIT_NORM / magnitude;
                chassis_vx *= scale;
                chassis_vy *= scale;
            }
            command->vx = chassis_vy;
            command->vy = -chassis_vx;
            command->wz = SPIN_WZ_NORM;
            command->enabled = true;
            return;
        }

        command->vx = chassis_vy;
        command->vy = -chassis_vx;
        command->wz = wz;
    } else {
        command->vx = vx;
        command->vy = vy;
        command->wz = wz;
    }

    command->enabled = (vx_raw != 0 || vy_raw != 0 || wz_raw != 0);
}

static void route_shooter(CommandRouter *router, const RemoteControlMessage *remote,
                          ShootCmd *command) {
    if (switch_is_down(remote->rc.s[0])) router->shooter_down_seen = true;
    bool switch_up = switch_is_up(remote->rc.s[0]);
    bool switch_mid = switch_is_mid(remote->rc.s[0]);
    command->friction_enabled = router->shooter_down_seen && (switch_up || switch_mid);
    command->feed_enabled = router->shooter_down_seen && switch_up;
}

static void route_gimbal(CommandRouter *router,
                         const CommandRouterInput *input,
                         uint32_t now_ms,
                         float dt_s,
                         GimbalCmd *command) {
    const RemoteControlMessage *remote = &input->remote;
    int16_t yaw_raw = apply_deadband((int16_t)-remote->rc.ch[0]);
    int16_t pitch_raw = apply_deadband(remote->rc.ch[1]);

    /* Reject an impossible single-frame yaw jump and retain the last input. */
    //if (fabsf(router->previous_yaw_input - (float)yaw_raw) > 1000.0f) {
        //yaw_raw = (int16_t)router->previous_yaw_input;
    //}

    const float max_input = (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
    float manual_yaw_rate = (float)yaw_raw / max_input;
    command->enabled = true;
    command->pitch_rate = (float)pitch_raw / max_input;

    if (router->spin_mode) {
        router->spin_hold_yaw_deg +=
            manual_yaw_rate * SPIN_GIMBAL_YAW_ADJ_DEG_PER_S * dt_s;
        command->yaw_rate = 0.0f;
        command->yaw_rate_memo = 1.0f;
        command->yaw_target_memo = router->spin_hold_yaw_deg;
    } else {
        command->yaw_rate = manual_yaw_rate;
        command->yaw_rate_memo = 0.0f;
        command->yaw_target_memo = 0.0f;
    }

    if (input->vision_updated) {
        command->vision_ts_ms = now_ms;
        if (input->vision.valid &&
            input->vision.target_state != VISION_TARGET_NONE) {
            command->vision_valid = true;
            command->vision_yaw_err_rad = input->vision.yaw_error_rad;
            command->vision_pitch_err_rad = input->vision.pitch_error_rad;
        } else {
            command->vision_valid = false;
            command->vision_yaw_err_rad = 0.0f;
            command->vision_pitch_err_rad = 0.0f;
        }
    } else if (command->vision_valid &&
               now_ms - command->vision_ts_ms > VISION_CMD_TIMEOUT_MS) {
        command->vision_valid = false;
    }

    router->previous_yaw_input = (float)yaw_raw;
}

void CommandRouter_Init(CommandRouter *router) {
    if (!router) {
        return;
    }
    memset(router, 0, sizeof(*router));
}

RobotStatus CommandRouter_Route(CommandRouter *router,
                                const CommandRouterInput *input,
                                uint32_t now_ms,
                                CommandRouterOutput *output) {
    if (!router || !input || !output) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }

    if (!input->remote_online) {
        router->shooter_down_seen = false;
        memset(output, 0, sizeof(*output));
        memset(&router->gimbal_memory, 0, sizeof(router->gimbal_memory));
        router->spin_mode = false;
        router->gimbal_follow_mode = false;
        router->route_time_valid = false;
        return ROBOT_STATUS_NOT_READY;
    }

    float dt_s = 0.001f;
    if (router->route_time_valid) {
        dt_s = (float)(uint32_t)(now_ms - router->previous_route_ms) / 1000.0f;
        if (dt_s <= 0.0f || dt_s > MAX_ROUTE_DT_S) dt_s = 0.001f;
    }
    router->previous_route_ms = now_ms;
    router->route_time_valid = true;

    bool follow_now = switch_is_mid(input->remote.rc.s[1]);
    bool spin_now = switch_is_up(input->remote.rc.s[1]);
    if (spin_now && !router->spin_mode) {
        router->spin_hold_yaw_deg = input->sensor.yaw_total_angle;
    }
    router->spin_mode = spin_now;
    router->gimbal_follow_mode = follow_now;

    memset(output, 0, sizeof(*output));
    route_chassis(input,
                  now_ms,
                  router->spin_mode,
                  router->gimbal_follow_mode,
                  &output->chassis);
    route_shooter(router, &input->remote, &output->shooter);
    route_gimbal(router, input, now_ms, dt_s, &router->gimbal_memory);
    output->gimbal = router->gimbal_memory;
    output->spin_mode = router->spin_mode;
    output->gimbal_follow_mode = router->gimbal_follow_mode;
    output->spin_hold_yaw_deg = router->spin_hold_yaw_deg;
    return ROBOT_STATUS_OK;
}
