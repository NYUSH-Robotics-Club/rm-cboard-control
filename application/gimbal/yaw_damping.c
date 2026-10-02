#include "yaw_damping.h"
#include <math.h>

static float smoothstep(float low, float high, float value) {
  float t = fmaxf(0.0f, fminf(1.0f, (value - low) / (high - low)));
  return t * t * (3.0f - 2.0f * t);
}

bool YawDamping_ConfigValid(const YawControlConfig *cfg) {
  if (!cfg) return false;
  float width = cfg->near_damping_blend_deg;
  float release = cfg->near_brake_release_rpm;
  float full = cfg->near_brake_full_rpm;
  return isfinite(width) && width >= 0.0f &&
      width <= cfg->near_error_deg &&
      cfg->near_error_deg + width <= cfg->approach_error_deg &&
      isfinite(release) && isfinite(full) &&
      ((release == 0.0f && full == 0.0f) || (release > 0.0f && full > release));
}

void YawDamping_Reset(YawDampingState *state) {
  *state = (YawDampingState){0};
}

float YawDamping_Update(YawDampingState *state, const YawControlConfig *cfg,
                        float error_deg, float actual_rpm, float target_ticks) {
  float error = fabsf(error_deg);
  float gain = error <= cfg->near_error_deg ? cfg->near_damping_gain :
      (error <= cfg->approach_error_deg ? cfg->approach_damping_gain :
                                        cfg->far_damping_gain);
  float width = cfg->near_damping_blend_deg;
  if (width > 0.0f && error <= cfg->near_error_deg + width) {
    float weight = smoothstep(cfg->near_error_deg - width,
                              cfg->near_error_deg + width, error);
    gain = cfg->near_damping_gain + weight *
        (cfg->approach_damping_gain - cfg->near_damping_gain);
  }

  /* 目标主动向原接近方向的反方向移动时重评估；机械越过固定目标不清状态。
   * 保持只在中/近区建立，远区不额外加阻尼，避免限制远距离跟踪。 */
  float speed = fabsf(actual_rpm);
  bool target_reversed = state->target_valid && state->holding &&
      (target_ticks - state->previous_target_ticks) * state->approach_direction < 0.0f;
  state->previous_target_ticks = target_ticks;
  state->target_valid = true;
  if (cfg->near_brake_full_rpm == 0.0f ||
      speed <= cfg->near_brake_release_rpm ||
      error > cfg->approach_error_deg || target_reversed ||
      actual_rpm * state->approach_direction < 0.0f) {
    state->holding = false;
  }
  if (cfg->near_brake_full_rpm > 0.0f &&
      speed > cfg->near_brake_release_rpm && error <= cfg->approach_error_deg &&
      error_deg * actual_rpm > 0.0f) {
    state->holding = true;
    state->approach_direction = actual_rpm > 0.0f ? 1.0f : -1.0f;
  }
  if (state->holding) {
    float weight = smoothstep(cfg->near_brake_release_rpm,
                              cfg->near_brake_full_rpm, speed);
    float brake_gain = fmaxf(cfg->near_damping_gain, cfg->approach_damping_gain);
    gain += weight * fmaxf(0.0f, brake_gain - gain);
  }
  return gain;
}
