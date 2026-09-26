/* Check safe RC loss behavior, mode routing, and time-based spin adjustment. */
#include <assert.h>
#include <math.h>
#include <string.h>
#include "command_router.h"
#include "chassis_strategy.h"
#include "infantry_standard.h"

static void test_encoder_follow(void)
{
    CommandRouter router;
    CommandRouter_Init(&router);
    CommandRouterInput input = {0};
    CommandRouterOutput output;
    input.remote_online = true;
    input.remote.rc.s[1] = RC_SW_MID;
    input.remote.rc.ch[3] = -660; /* Keep the existing RC forward polarity. */
    input.encoder_follow = input.yaw_heading_valid = true;
    input.yaw_feedback_ms = 1000;
    input.sensor.c_yaw = -37;
    input.sensor.yaw_total_angle = 153; /* Must not affect encoder follow. */
    const OmniChassisConfig *geometry = g_robot_config_infantry_standard.omni;
    for (int degrees = -180; degrees <= 180; degrees += 5) {
        input.yaw_relative_deg = (float)degrees;
        assert(CommandRouter_Route(&router, &input, 1000, &output) == ROBOT_STATUS_OK);
        assert(output.gimbal_follow_mode && output.chassis.enabled);
        float angle = degrees * 0.0174532925199433f;
        assert(fabsf(output.chassis.vx + cosf(angle)) < 0.0001f);
        assert(fabsf(output.chassis.vy - sinf(angle)) < 0.0001f);
        assert(output.chassis.wz == 0);

        ChassisKinematicsInput kin = {
            .vx = output.chassis.vx, .vy = output.chassis.vy,
            .wz = output.chassis.wz, .omni = geometry, .max_drive_speed = 8050
        };
        ChassisKinematicsOutput wheels;
        assert(OmniChassisStrategy_Get()->compute(&kin, &wheels) == ROBOT_STATUS_OK);
        assert(wheels.drive_count == 4);
        /* Independently recover body translation from wheel surface speeds.
         * This checks the composed router/kinematics result across a full turn.
         */
        float xx = 0, xy = 0, yy = 0, bx = 0, by = 0;
        for (unsigned i = 0; i < 4; ++i) {
            const OmniWheelConfig *w = &geometry->wheels[i];
            float speed = wheels.drive_speed[i] * (6.28318530718f / 60)
                        * w->radius_m / w->reduction_ratio;
            xx += w->drive_x * w->drive_x;
            xy += w->drive_x * w->drive_y;
            yy += w->drive_y * w->drive_y;
            bx += w->drive_x * speed;
            by += w->drive_y * speed;
        }
        float determinant = xx * yy - xy * xy;
        assert(determinant > 0.01f);
        float vx = (yy * bx - xy * by) / determinant;
        float vy = (xx * by - xy * bx) / determinant;
        assert(fabsf(vx - geometry->max_translation_mps * cosf(angle)) < 0.0001f);
        assert(fabsf(vy - geometry->max_translation_mps * sinf(angle)) < 0.0001f);
    }
    input.yaw_relative_deg = 90;
    input.remote.rc.ch[3] = 0;
    input.remote.rc.ch[2] = 660;
    input.remote.rc.ch[4] = 330;
    CommandRouter_Route(&router, &input, 1000, &output);
    assert(fabsf(output.chassis.vx + 1) < 0.0001f);
    assert(fabsf(output.chassis.vy) < 0.0001f && output.chassis.wz == 0.5f);
    CommandRouter_Route(&router, &input, 1021, &output);
    assert(!output.chassis.enabled && output.chassis.vx == 0 && output.chassis.wz == 0);
    input.yaw_feedback_ms = UINT32_MAX - 9;
    CommandRouter_Route(&router, &input, 5, &output);
    assert(output.chassis.enabled); /* Unsigned timestamp wrap, age 15 ms. */
    input.yaw_relative_deg = NAN;
    CommandRouter_Route(&router, &input, 5, &output);
    assert(!output.chassis.enabled);
    input.remote.rc.s[1] = RC_SW_DOWN;
    CommandRouter_Route(&router, &input, 5, &output);
    assert(output.chassis.enabled && output.chassis.vx == 0 && output.chassis.vy == 1);
    input.remote_online = false;
    CommandRouter_Route(&router, &input, 5, &output);
    assert(!output.chassis.enabled && !output.gimbal.enabled);
}

int main(void)
{
    test_encoder_follow();
    CommandRouter router;
    CommandRouterInput input;
    CommandRouterOutput output;
    memset(&input, 0, sizeof(input));
    CommandRouter_Init(&router);

    assert(CommandRouter_Route(&router, &input, 100U, &output) ==
           ROBOT_STATUS_NOT_READY);
    assert(!output.chassis.enabled && !output.gimbal.enabled &&
           !output.shooter.feed_enabled);

    input.remote_online = true;
    input.remote.rc.s[0] = RC_SW_UP;
    input.remote.rc.s[1] = RC_SW_UP;
    input.remote.rc.ch[0] = -660;
    input.sensor.yaw_total_angle = 10.0f;
    assert(CommandRouter_Route(&router, &input, 1000U, &output) ==
           ROBOT_STATUS_OK);
    assert(output.spin_mode && output.chassis.enabled);
    assert(!output.shooter.feed_enabled && !output.shooter.friction_enabled);
    const float first_target = output.spin_hold_yaw_deg;
    assert(first_target == input.sensor.yaw_total_angle);

    assert(CommandRouter_Route(&router, &input, 1010U, &output) ==
           ROBOT_STATUS_OK);
    assert(output.spin_hold_yaw_deg > first_target + 1.0f);
    assert(output.spin_hold_yaw_deg < first_target + 1.3f);

    input.remote.rc.s[0] = RC_SW_DOWN;
    assert(CommandRouter_Route(&router, &input, 1020U, &output) == ROBOT_STATUS_OK);
    input.remote.rc.s[0] = RC_SW_UP;
    assert(CommandRouter_Route(&router, &input, 1030U, &output) == ROBOT_STATUS_OK);
    assert(output.shooter.feed_enabled && output.shooter.friction_enabled);

    input.remote_online = false;
    assert(CommandRouter_Route(&router, &input, 1300U, &output) ==
           ROBOT_STATUS_NOT_READY);
    assert(!output.chassis.enabled && !output.gimbal.enabled);
    input.remote_online = true;
    assert(CommandRouter_Route(&router, &input, 1400U, &output) == ROBOT_STATUS_OK);
    assert(!output.shooter.feed_enabled && !output.shooter.friction_enabled);
    return 0;
}
