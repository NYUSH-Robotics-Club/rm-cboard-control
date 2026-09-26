/*
 * Project body velocity onto each omni wheel's driven direction.
 * Normalized input becomes m/s and rad/s, then rotor RPM in configured wheel order.
 * Missing or invalid geometry leaves every output zero; no hardware is accessed.
 */
#include "chassis_strategy.h"
#include <math.h>
#include <string.h>

#define RAD_PER_SEC_TO_RPM 9.54929658551372f

/* Check that translation in both axes and rotation are independently possible.
 * Normalize lever arms so the rank tolerance does not depend on metres.
 */
static bool has_three_dof(float rows[OMNI_WHEEL_COUNT][3], float max_lever)
{
    if (max_lever <= 0.0f) return false;
    for (unsigned i = 0; i < OMNI_WHEEL_COUNT; ++i) rows[i][2] /= max_lever;
    for (unsigned a = 0; a < 2U; ++a) {
        for (unsigned b = a + 1U; b < 3U; ++b) {
            for (unsigned c = b + 1U; c < 4U; ++c) {
                float det = rows[a][0] * (rows[b][1]*rows[c][2] - rows[b][2]*rows[c][1])
                          - rows[a][1] * (rows[b][0]*rows[c][2] - rows[b][2]*rows[c][0])
                          + rows[a][2] * (rows[b][0]*rows[c][1] - rows[b][1]*rows[c][0]);
                if (fabsf(det) > 0.0001f) return true;
            }
        }
    }
    return false;
}

static RobotStatus omni_compute(const ChassisKinematicsInput *input,
                                ChassisKinematicsOutput *output)
{
    if (!output) return ROBOT_STATUS_INVALID_ARGUMENT;
    memset(output, 0, sizeof(*output));
    if (!input) return ROBOT_STATUS_INVALID_ARGUMENT;
    const OmniChassisConfig *geometry = input->omni;
    if (!geometry || !geometry->configured) return ROBOT_STATUS_NOT_READY;
    if (!isfinite(input->vx) || !isfinite(input->vy) || !isfinite(input->wz) ||
        !isfinite(input->max_drive_speed) || input->max_drive_speed < 0.0f ||
        !isfinite(geometry->max_translation_mps) || geometry->max_translation_mps <= 0.0f ||
        !isfinite(geometry->max_rotation_radps) || geometry->max_rotation_radps <= 0.0f) {
        return ROBOT_STATUS_INVALID_ARGUMENT;
    }

    /* Preserve diagonal direction while limiting the translation magnitude.
     * Scaling by the largest component avoids overflow for malformed input.
     */
    float largest = fmaxf(1.0f, fmaxf(fabsf(input->vx), fabsf(input->vy)));
    float vx = input->vx / largest, vy = input->vy / largest;
    float magnitude = sqrtf(vx*vx + vy*vy);
    if (magnitude > 1.0f) { vx /= magnitude; vy /= magnitude; }
    vx *= geometry->max_translation_mps;
    vy *= geometry->max_translation_mps;
    float wz = fmaxf(-5.0f, fminf(5.0f, input->wz)) * geometry->max_rotation_radps;

    float rows[OMNI_WHEEL_COUNT][3];
    float speeds[OMNI_WHEEL_COUNT];
    float max_lever = 0.0f, peak = 0.0f;
    for (unsigned i = 0; i < OMNI_WHEEL_COUNT; ++i) {
        const OmniWheelConfig *wheel = &geometry->wheels[i];
        if (!isfinite(wheel->x_m) || !isfinite(wheel->y_m) ||
            !isfinite(wheel->drive_x) || !isfinite(wheel->drive_y) ||
            !isfinite(wheel->radius_m) || wheel->radius_m <= 0.0f ||
            !isfinite(wheel->reduction_ratio) || wheel->reduction_ratio <= 0.0f ||
            fabsf(wheel->drive_x*wheel->drive_x + wheel->drive_y*wheel->drive_y - 1.0f) > 0.001f) {
            return ROBOT_STATUS_INVALID_ARGUMENT;
        }
        for (unsigned j = 0; j < i; ++j) {
            if (geometry->wheels[j].motor_id == wheel->motor_id)
                return ROBOT_STATUS_INVALID_ARGUMENT;
        }
        float lever = wheel->x_m * wheel->drive_y - wheel->y_m * wheel->drive_x;
        rows[i][0] = wheel->drive_x;
        rows[i][1] = wheel->drive_y;
        rows[i][2] = lever;
        max_lever = fmaxf(max_lever, fabsf(lever));
        speeds[i] = (wheel->drive_x*vx + wheel->drive_y*vy + lever*wz)
                  / wheel->radius_m * wheel->reduction_ratio * RAD_PER_SEC_TO_RPM;
        if (!isfinite(lever) || !isfinite(speeds[i])) return ROBOT_STATUS_INVALID_ARGUMENT;
        peak = fmaxf(peak, fabsf(speeds[i]));
    }
    if (!has_three_dof(rows, max_lever)) return ROBOT_STATUS_INVALID_ARGUMENT;

    /* Scale all wheels together: independent clipping changes the motion. */
    float scale = peak > input->max_drive_speed ? input->max_drive_speed / peak : 1.0f;
    for (unsigned i = 0; i < OMNI_WHEEL_COUNT; ++i) output->drive_speed[i] = speeds[i] * scale;
    output->drive_count = OMNI_WHEEL_COUNT;
    return ROBOT_STATUS_OK;
}

const ChassisStrategy *OmniChassisStrategy_Get(void)
{
    static const ChassisStrategy strategy = {
        .type = CHASSIS_TYPE_OMNI,
        .name = "omni-four-wheel",
        .execution = CHASSIS_EXECUTION_KINEMATICS,
        .implemented = true,
        .compute = omni_compute
    };
    return &strategy;
}
