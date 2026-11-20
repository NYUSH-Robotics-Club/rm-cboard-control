#include "pitch_control_math.h"
#include <math.h>

bool PitchControl_ConfigValid(const PitchControlConfig *c) {
  if (!c || !isfinite(c->manual_rate_deg_s) || c->manual_rate_deg_s <= 0 ||
      !isfinite(c->manual_stick_gain) || c->manual_stick_gain <= 0 ||
      !isfinite(c->velocity_damping_gain) || c->velocity_damping_gain < 0 ||
      !isfinite(c->load_gain) || c->load_gain < 0 || c->load_gain > 1 ||
      !isfinite(c->load_output_max) || c->load_output_max < 0) return false;
  if (c->load_gain == 0 || c->load_output_max == 0) return true;
  return isfinite(c->load_reference_ticks) && isfinite(c->load_bias) &&
      isfinite(c->load_slope_per_tick) && isfinite(c->load_min_ticks) &&
      isfinite(c->load_max_ticks) && c->load_min_ticks < c->load_max_ticks;
}

float PitchControl_Load(const PitchControlConfig *c, float position) {
  if (c->load_gain==0 || c->load_output_max==0) return 0;
  float q=fmaxf(c->load_min_ticks,fminf(position,c->load_max_ticks));
  float value=c->load_gain*(c->load_bias+c->load_slope_per_tick*(q-c->load_reference_ticks));
  return fmaxf(-c->load_output_max,fminf(value,c->load_output_max));
}
