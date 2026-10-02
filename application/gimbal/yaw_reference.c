/*
 * 以相邻反馈的最短增量展开位置，目标与位置共用连续坐标。
 * 失去连续性时作废，不根据过期反馈猜圈数。
 */
#include "yaw_reference.h"
#include <math.h>

float YawReference_Wrap(float ticks) {
    float raw = fmodf(ticks, YAW_ENCODER_TICKS);
    return raw < 0.0f ? raw + YAW_ENCODER_TICKS : raw;
}

void YawReference_Seed(YawReference *ref, uint16_t raw,
                       uint32_t feedback_ms, uint32_t now_ms) {
    *ref = (YawReference){.valid = raw < 8192U, .last_raw = raw,
        .feedback_ms = feedback_ms, .control_ms = now_ms,
        .position_ticks = raw, .target_ticks = raw};
}

bool YawReference_Update(YawReference *ref, uint16_t raw, int16_t rpm,
                         uint32_t feedback_ms, uint32_t now_ms, float *dt_s) {
    uint32_t elapsed = now_ms - ref->control_ms;
    uint32_t feedback_elapsed = feedback_ms - ref->feedback_ms;
    *dt_s = 0.0f;
    if (!ref->valid || raw >= 8192U || elapsed > YAW_REFERENCE_MAX_GAP_MS ||
        now_ms - feedback_ms > YAW_REFERENCE_MAX_GAP_MS ||
        feedback_elapsed > YAW_REFERENCE_MAX_GAP_MS ||
        fabsf((float)rpm) >= YAW_REFERENCE_MAX_SPEED_RPM) {
        ref->valid = false;
        return false;
    }
    float delta = (float)raw - ref->last_raw;
    if (delta > 4096.0f) delta -= YAW_ENCODER_TICKS;
    if (delta < -4096.0f) delta += YAW_ENCODER_TICKS;
    /* 毫秒时间戳可重用；留1ms量化裕量与少量编码器抖动裕量。 */
    float allowed = YAW_REFERENCE_MAX_SPEED_RPM * YAW_ENCODER_TICKS / 60000.0f *
                    (float)(feedback_elapsed + 1U) + 4.0f;
    if (fabsf(delta) > allowed) {
        ref->valid = false;
        return false;
    }
    ref->position_ticks += delta;
    ref->last_raw = raw;
    ref->feedback_ms = feedback_ms;
    ref->control_ms = now_ms;
    *dt_s = (float)elapsed * 0.001f;
    /* 同时平移两个坐标，防止长时间转动使float丢失单刻度精度。 */
    if (fabsf(ref->position_ticks) >= 65536.0f) {
        float offset = floorf(ref->position_ticks / YAW_ENCODER_TICKS) * YAW_ENCODER_TICKS;
        ref->position_ticks -= offset;
        ref->target_ticks -= offset;
    }
    return true;
}

void YawReference_Advance(YawReference *ref, float stick, float rate_deg_s,
                          float dt_s, float input_gain) {
    if (!ref->valid || stick == 0.0f || dt_s <= 0.0f) return;
    if (!isfinite(rate_deg_s) || rate_deg_s <= 0.0f ||
        !isfinite(input_gain) || input_gain <= 0.0f) return;
    float step = fmaxf(-1.0f, fminf(stick, 1.0f)) * input_gain * rate_deg_s * dt_s *
                 YAW_ENCODER_TICKS / 360.0f;
    ref->target_ticks += step;
}
