#include "yaw_damping.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void close_to(float actual, float expected) {
  assert(fabsf(actual - expected) < 0.0001f);
}
int main(void) {
  YawControlConfig cfg = {.near_error_deg=8, .approach_error_deg=45,
      .near_damping_gain=.60f, .approach_damping_gain=.85f,
      .far_damping_gain=.60f, .near_damping_blend_deg=3,
      .near_brake_release_rpm=8, .near_brake_full_rpm=20};
  YawDampingState state = {0};
  assert(YawDamping_ConfigValid(&cfg));
  close_to(YawDamping_Update(&state,&cfg,5,0,100),.60f);
  close_to(YawDamping_Update(&state,&cfg,8,0,100),.725f);
  close_to(YawDamping_Update(&state,&cfg,11,0,100),.85f);
  float previous=.60f;
  for (int i=0;i<=600;i++) {
    float gain=YawDamping_Update(&state,&cfg,5+i*.01f,0,100);
    assert(gain>=previous && gain-previous<.001f);
    previous=gain;
  }
  /* 高速进近，误差过零后继续制动，8~20RPM连续释放。 */
  close_to(YawDamping_Update(&state,&cfg,10,35,100),.85f);
  close_to(YawDamping_Update(&state,&cfg,2,25,100),.85f);
  close_to(YawDamping_Update(&state,&cfg,-1,22,100),.85f);
  assert(state.holding);
  close_to(YawDamping_Update(&state,&cfg,-2,14,100),.725f);
  close_to(YawDamping_Update(&state,&cfg,-3,8,100),.60f);
  assert(!state.holding);
  /* 小角度低速跟随不保持，负方向行为对称。 */
  close_to(YawDamping_Update(&state,&cfg,2,6,100),.60f);
  close_to(YawDamping_Update(&state,&cfg,-2,-25,100),.85f);
  close_to(YawDamping_Update(&state,&cfg,1,-25,100),.85f);
  /* 主动反向目标在身后时退出；再次接近新目标可重新建立。 */
  close_to(YawDamping_Update(&state,&cfg,3,-25,110),.60f);
  assert(!state.holding);
  close_to(YawDamping_Update(&state,&cfg,3,25,110),.85f);
  YawDamping_Reset(&state); /* 模式/失联/旁路共同使用的重置入口。 */
  close_to(YawDamping_Update(&state,&cfg,-2,25,110),.60f);
  assert(!state.holding);
  close_to(YawDamping_Update(&state,&cfg,60,50,200),.60f);
  assert(!state.holding);
  cfg.near_brake_full_rpm=8;
  assert(!YawDamping_ConfigValid(&cfg));
  cfg.near_brake_full_rpm=NAN;
  assert(!YawDamping_ConfigValid(&cfg));
  cfg.near_brake_full_rpm=cfg.near_brake_release_rpm=0;
  cfg.near_damping_blend_deg=0;
  assert(YawDamping_ConfigValid(&cfg));
  close_to(YawDamping_Update(&state,&cfg,8,30,200),.60f);
  close_to(YawDamping_Update(&state,&cfg,8.01f,30,200),.85f);
  cfg.near_damping_blend_deg=9;
  assert(!YawDamping_ConfigValid(&cfg));
  puts("yaw damping tests passed");
}
