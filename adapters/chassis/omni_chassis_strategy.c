
#include "chassis_strategy.h"
#include <math.h>
#include <string.h>
#define OMNI_WHEEL_COUNT 4U
#define SIN_COS_45 0.70710678f

static const struct {
    float dx;   /* d 的 x 分量 = -sin(alpha)，无量纲 */
    float dy;   /* d 的 y 分量 =  cos(alpha)，无量纲 */
} k_wheel[OMNI_WHEEL_COUNT] = {
    { -SIN_COS_45,  SIN_COS_45 },   /* [0] FL, alpha =  45° */
    {  SIN_COS_45,  SIN_COS_45 },   /* [1] FR, alpha = 315° */
    { -SIN_COS_45, -SIN_COS_45 },   /* [2] BL, alpha = 135° */
    {  SIN_COS_45, -SIN_COS_45 },   /* [3] BR, alpha = 225° */
};
static RobotStatus omni_compute(const ChassisKinematicsInput *input,
                                ChassisKinematicsOutput *output)
{
    if (!output) return ROBOT_STATUS_INVALID_ARGUMENT;
    memset(output, 0, sizeof(*output));   /* 出错时输出恒为零 */
    if (!input) return ROBOT_STATUS_INVALID_ARGUMENT;

    /* 写成 !(a <= x && x <= b) 而不是 (x < a || x > b)：NaN 的比较全为假，
     * 这样 NaN 也会落进拒绝分支，不需要 <math.h> 的 isfinite。 */
    if (!(-1.0f <= input->vx && input->vx <= 1.0f) ||
        !(-1.0f <= input->vy && input->vy <= 1.0f) ||
        !(-1.0f <= input->wz && input->wz <= 1.0f) ||
        !(input->max_drive_speed >= 0.0f)) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }

    /* n_i = max_drive_speed · (vx·dx_i + vy·dy_i + wz)    [转子 rpm] */
    float speed[OMNI_WHEEL_COUNT];
    float peak = 0.0f;
    for (unsigned i = 0; i < OMNI_WHEEL_COUNT; ++i) {
        float u = input->vx * k_wheel[i].dx
                + input->vy * k_wheel[i].dy
                + input->wz;
        speed[i] = input->max_drive_speed * u;
        float mag = speed[i] < 0.0f ? -speed[i] : speed[i];
        if (mag > peak) peak = mag;
    }

    /* 三项叠加最坏是 (√2+1)·max。超限时四轮等比缩，保持运动方向不变；
     * 逐轮独立截断会改变合成运动，不能用。 */
    float scale = 1.0f;
    if (peak > input->max_drive_speed && peak > 0.0f) {
        scale = input->max_drive_speed / peak;
    }
    for (unsigned i = 0; i < OMNI_WHEEL_COUNT; ++i) {
        output->drive_speed[i] = speed[i] * scale;
    }
    output->drive_count = OMNI_WHEEL_COUNT;
    output->steer_count = 0U;
    return ROBOT_STATUS_OK;
}

const ChassisStrategy *OmniChassisStrategy_Get(void)
{
    static const ChassisStrategy strategy = {
        .type = CHASSIS_TYPE_OMNI,
        .name = "omni-4x45deg-assumed",
        .execution = CHASSIS_EXECUTION_KINEMATICS,
        .implemented = true,
        .compute = omni_compute
    };
    return &strategy;
}