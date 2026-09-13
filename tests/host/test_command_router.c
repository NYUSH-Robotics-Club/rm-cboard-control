/* Check safe RC loss behavior, mode routing, and time-based spin adjustment. */
#include <assert.h>
#include <math.h>
#include <string.h>
#include "command_router.h"

int main(void)
{
    CommandRouter router;
    CommandRouterInput input;
    CommandRouterOutput output;
    memset(&input, 0, sizeof(input));
    CommandRouter_Init(&router);

    assert(CommandRouter_Route(&router, &input, 100U, &output) ==
           ROBOT_STATUS_NOT_READY);
    assert(!output.chassis.enabled && !output.gimbal.enabled &&
           !output.shooter.feed_enabled);

    /* 普通模式下逐个检查底盘通道；归一化输出无单位，满杆为 +/-1。 */
    input.remote_online = true;
    input.remote.rc.s[0] = RC_SW_DOWN;
    input.remote.rc.s[1] = RC_SW_DOWN;
    const unsigned channels[] = {3U, 2U, 4U};
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        for (int sign = -1; sign <= 1; sign += 2) {
            memset(input.remote.rc.ch, 0, sizeof(input.remote.rc.ch));
            input.remote.rc.ch[channels[axis]] = (int16_t)(sign * 660);
            assert(CommandRouter_Route(&router, &input, 200U, &output) ==
                   ROBOT_STATUS_OK);
            const float expected = (float)(axis == 2U ? sign : -sign);
            assert(output.chassis.enabled);
            assert(output.chassis.vx == (axis == 0U ? expected : 0.0f));
            assert(output.chassis.vy == (axis == 1U ? expected : 0.0f));
            assert(output.chassis.wz == (axis == 2U ? expected : 0.0f));
            assert(!output.shooter.feed_enabled && !output.shooter.friction_enabled);
        }
    }

    /* 摇杆仍有偏转时失联必须清除运动指令，不能保留上一帧。 */
    input.remote_online = false;
    assert(CommandRouter_Route(&router, &input, 500U, &output) ==
           ROBOT_STATUS_NOT_READY);
    assert(!output.chassis.enabled && output.chassis.vx == 0.0f &&
           output.chassis.vy == 0.0f && output.chassis.wz == 0.0f);

    memset(input.remote.rc.ch, 0, sizeof(input.remote.rc.ch));
    input.remote_online = true;
    input.remote.rc.ch[2] = 2;
    input.remote.rc.ch[3] = -2;
    assert(CommandRouter_Route(&router, &input, 600U, &output) == ROBOT_STATUS_OK);
    assert(!output.chassis.enabled && output.chassis.vx == 0.0f &&
           output.chassis.vy == 0.0f && output.chassis.wz == 0.0f);
    memset(input.remote.rc.ch, 0, sizeof(input.remote.rc.ch));

    input.remote_online = true;
    input.remote.rc.s[0] = RC_SW_UP;
    input.remote.rc.s[1] = RC_SW_UP;
    input.remote.rc.ch[0] = -660;
    input.sensor.yaw_total_angle = 10.0f;
    assert(CommandRouter_Route(&router, &input, 1000U, &output) ==
           ROBOT_STATUS_OK);
    assert(output.spin_mode && output.chassis.enabled);
    assert(output.shooter.feed_enabled && output.shooter.friction_enabled);
    const float first_target = output.spin_hold_yaw_deg;

    assert(CommandRouter_Route(&router, &input, 1010U, &output) ==
           ROBOT_STATUS_OK);
    assert(output.spin_hold_yaw_deg > first_target + 1.0f);
    assert(output.spin_hold_yaw_deg < first_target + 1.3f);

    input.remote_online = false;
    assert(CommandRouter_Route(&router, &input, 1300U, &output) ==
           ROBOT_STATUS_NOT_READY);
    assert(!output.chassis.enabled && !output.gimbal.enabled);
    return 0;
}
