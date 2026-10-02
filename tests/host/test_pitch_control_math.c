#include "pitch_control_math.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
 PitchControlConfig c={.manual_rate_deg_s=1200,.manual_stick_gain=.3f,
  .velocity_damping_gain=.6f,
  .load_reference_ticks=1971,.load_bias=1016.2475f,.load_slope_per_tick=14.942652f,
  .load_min_ticks=1656,.load_max_ticks=2318,.load_gain=.75f,.load_output_max=6000};
 assert(PitchControl_ConfigValid(&c));
 assert(fabsf(PitchControl_Load(&c,1971)-762.1856f)<.01f);
 assert(PitchControl_Load(&c,1607)==PitchControl_Load(&c,1656));
 assert(PitchControl_Load(&c,2374)==PitchControl_Load(&c,2318));
 c.load_output_max=100;assert(PitchControl_Load(&c,2318)==100);
 c.load_gain=0;assert(PitchControl_Load(&c,1971)==0);
 c.velocity_damping_gain=-1;assert(!PitchControl_ConfigValid(&c));
 c.velocity_damping_gain=.6f;c.load_gain=1;c.load_slope_per_tick=NAN;
 assert(!PitchControl_ConfigValid(&c));
 puts("pitch math: PASS (valid config, disabled load, bounded fit)");
}
