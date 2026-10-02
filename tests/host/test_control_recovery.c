/* Real application callbacks: stale feedback and disable must erase old motor commands. */
#include "gimbal_controller.h"
#include "gimbal_monitor.h"
#include "pitch_control_math.h"
#include "shooter_controller.h"
#include "motor_driver.h"
#include "motor_service.h"
#include "message_center.h"
#include "infantry_standard.h"
#include "logger.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint32_t now_ms;
static MotorContext_t motors[16];
static int16_t outputs[16];
static float setpoints[16];
static bool can_armed = true;
static RobotStatus monitor_command_status = ROBOT_STATUS_OK;
MotorCommandUnit MotorService_GetCommandUnit(uint8_t id) {
 return id == 5 ? MOTOR_COMMAND_UNIT_CURRENT_COUNTS : MOTOR_COMMAND_UNIT_VOLTAGE_COUNTS;
}
bool BspCan_OutputsArmed(void) { return can_armed; }
void ShooterApp_Init(void);
uint32_t BspTime_NowMs(void){return now_ms;}
uint32_t BspTime_NowUs(void){return now_ms*1000U;}
void BspTime_DelayMs(uint32_t ms){now_ms+=ms;}
void Logger_Log(LogTag_t t,LogLevel_t l,const char *f,...){(void)t;(void)l;(void)f;}
void Logger_CSV(LogTag_t t,const char *f,...){(void)t;(void)f;}
void USB_CDC_Printf(const char *f,...){(void)f;}
const MotorConfig_t *MotorService_GetConfig(uint8_t id){
 const RobotConfig_t *r=&g_robot_config_infantry_standard;
 for(uint8_t i=0;i<r->total_motor_count;i++)if(r->motor_configs[i].motor_id==id)return &r->motor_configs[i];
 return NULL;
}
uint8_t MotorService_FindByRole(MotorRole_e role,uint8_t *ids,uint8_t cap){
 uint8_t count=0;
 for(uint8_t id=1;id<10&&count<cap;id++)if(MotorService_GetConfig(id)->role==role)ids[count++]=id;
 return count;
}
MotorContext_t *MotorDriver_GetContext(uint8_t id){return id<16?&motors[id]:NULL;}
/* 使用当前车型的 yaw 电流刻度上限，避免调参后仍只验证旧的限流值。 */
int16_t MotorDriver_GetCommandLimit(uint8_t id){
 return id==5?MotorService_GetConfig(id)->protocol.dji.command_limit:25000;
}
RobotStatus MotorService_CommandCurrent(uint8_t id,int16_t value){assert(id<16);outputs[id]=value;return monitor_command_status;}
RobotStatus MotorService_CommandConfigured(uint8_t id,float target,int16_t value){setpoints[id]=target;return MotorService_CommandCurrent(id,value);}
/* 三台反馈联锁走真实消息回调；依次模拟每台断线和首帧缺失。 */
static void shooter_feedback(uint8_t id, uint32_t tick) {
 MotorFeedbackEvent feedback = {.id=id, .speed=1000, .tick_ms=tick};
 assert(MsgCenter_Publish(TOPIC_MOTOR_FEEDBACK, &feedback, sizeof(feedback)) == 0);
}
static void shooter_command(void) {
 ShootCmd command = {.friction_enabled=true, .feed_enabled=true};
 assert(MsgCenter_Publish(TOPIC_SHOOT_CMD, &command, sizeof(command)) == 0);
 MsgCenter_Dispatch();
}
static void assert_shooter_stopped(void) {
 const ShooterController *controller = ShooterApp_GetController();
 assert(outputs[6] == 0 && outputs[7] == 0 && outputs[9] == 0);
 assert(controller->ramped_turntable == 0 && controller->ramped_shooter1 == 0 && controller->ramped_shooter2 == 0);
 assert(controller->turntable_target == 0 && controller->shooter1_target == 0 && controller->shooter2_target == 0);
 const PID_Controller *pids[] = {&controller->turntable_pid, &controller->shooter1_pid, &controller->shooter2_pid};
 for (unsigned i=0; i<3; ++i) {
  assert(pids[i]->integral == 0 && pids[i]->iout == 0 && pids[i]->dout == 0);
  assert(pids[i]->output == 0 && pids[i]->last_output == 0 && pids[i]->last_dout == 0);
  assert(pids[i]->error[0] == 0 && pids[i]->error[1] == 0 && pids[i]->error[2] == 0);
 }
 assert(!ShooterController_IsRunning(controller));
}
static void test_shooter_interlock(void) {
 MsgEvent queue[32];
 const uint8_t ids[] = {6,7,9};
 can_armed = true;
 for (unsigned missing=0; missing<3; ++missing) {
  now_ms=0;
  MsgCenter_Init(queue,32); ShooterApp_Init();
  for (unsigned i=0; i<3; ++i) if(i!=missing) shooter_feedback(ids[i], now_ms);
  shooter_command(); assert_shooter_stopped();
  assert(ShooterApp_GetController()->feedback_fault);
  /* 时间戳0也是真实首帧；补齐后可从零启动。 */
  shooter_feedback(ids[missing], now_ms); shooter_command();
  assert(setpoints[6]==-500 && setpoints[7]==500 && setpoints[9]==500);
  assert(!ShooterApp_GetController()->feedback_fault);
  now_ms=1000;
  for (unsigned i=0; i<3; ++i) shooter_feedback(ids[i], now_ms);
  for (unsigned i=0; i<15; ++i) shooter_command();
  assert(setpoints[6]==-7500 && setpoints[7]==7500 && setpoints[9]==1000);
  now_ms=1100;
  for (unsigned i=0; i<3; ++i) if(i!=missing) shooter_feedback(ids[i], now_ms);
  shooter_command(); assert(!ShooterApp_GetController()->feedback_fault);
  now_ms=1101; shooter_command(); assert_shooter_stopped();
  assert(ShooterApp_GetController()->feedback_fault);
  for (unsigned i=0; i<20; ++i) shooter_command();
  assert_shooter_stopped();
  now_ms=1102;
  for (unsigned i=0; i<3; ++i) shooter_feedback(ids[i], now_ms);
  shooter_command();
  assert(setpoints[6]==-500 && setpoints[7]==500 && setpoints[9]==500);
  can_armed=false; shooter_command(); assert_shooter_stopped(); can_armed=true;
 }
 /* uint32毫秒回绕不能误判新鲜反馈。 */
 now_ms=UINT32_MAX-50U;
 for(unsigned i=0; i<3; ++i) shooter_feedback(ids[i],now_ms);
 shooter_command(); now_ms=20U; shooter_command();
 assert(!ShooterApp_GetController()->feedback_fault);
 now_ms=50U; shooter_command(); assert_shooter_stopped();
}

/* 用真实命令/反馈回调验证堵转持续时间和发流方向，电流均为原始刻度。 */
static void feed_step(uint32_t tick, int16_t current, bool enabled) {
 now_ms=tick;
 shooter_feedback(6,tick); shooter_feedback(7,tick);
 MotorFeedbackEvent feedback={.id=9,.current=current,.speed=0,.tick_ms=tick};
 assert(MsgCenter_Publish(TOPIC_MOTOR_FEEDBACK,&feedback,sizeof(feedback))==0);
 ShootCmd command={.friction_enabled=true,.feed_enabled=enabled};
 assert(MsgCenter_Publish(TOPIC_SHOOT_CMD,&command,sizeof(command))==0);
 MsgCenter_Dispatch();
}

static void trigger_feed_reverse(uint32_t start, int16_t current) {
 for(uint32_t elapsed=0;elapsed<FEED_STALL_DURATION_MS;elapsed+=10U) {
  feed_step(start+elapsed,current,true);
  assert(setpoints[9]>0 && outputs[9]>0);
 }
 feed_step(start+FEED_STALL_DURATION_MS,current,true);
 assert(setpoints[9]==-FEED_REVERSE_SPEED_RPM && outputs[9]<0);
 assert(ShooterApp_GetController()->feed_recovery.state==FEED_RECOVERY_REVERSING);
}

static void test_feed_stall_recovery(void) {
 MsgEvent queue[32];
 can_armed=true;
 MsgCenter_Init(queue,32); ShooterApp_Init();
 /* 正常电流和短脉冲不触发；低于阈值立即重新计时。 */
 for(uint32_t t=0;t<=1000;t+=10) feed_step(t,g_feed_stall_current_threshold-1,true);
 for(uint32_t t=1010;t<1300;t+=10) feed_step(t,g_feed_stall_current_threshold,true);
 feed_step(1300,0,true);
 assert(!ShooterApp_GetController()->feed_recovery.reverse_attempted);
 trigger_feed_reverse(1310,g_feed_stall_current_threshold);
 uint32_t reverse_at=1310+FEED_STALL_DURATION_MS;
 for(uint32_t t=10;t<FEED_REVERSE_DURATION_MS;t+=10) {
  feed_step(reverse_at+t,INT16_MIN,true);
  assert(setpoints[9]==-FEED_REVERSE_SPEED_RPM && outputs[9]<0);
  assert(setpoints[6]==-SHOOTER_CONST_SPEED && setpoints[7]==SHOOTER_CONST_SPEED);
 }
 feed_step(reverse_at+FEED_REVERSE_DURATION_MS,0,true);
 assert(setpoints[9]==0 && outputs[9]==0);
 feed_step(reverse_at+FEED_REVERSE_DURATION_MS+10,0,true);
 assert(setpoints[9]==SHOOTER_RAMP_STEP && outputs[9]>0);

 /* 同一次拨弹重堵只停拨盘，不反复换向，摩擦轮维持原目标。 */
 uint32_t retry_at=reverse_at+FEED_REVERSE_DURATION_MS+20;
 for(uint32_t t=0;t<=FEED_STALL_DURATION_MS;t+=10) feed_step(retry_at+t,INT16_MIN,true);
 assert(ShooterApp_GetController()->feed_recovery.state==FEED_RECOVERY_BLOCKED);
 assert(outputs[9]==0 && setpoints[9]==0);
 for(uint32_t t=10;t<=1000;t+=10) {
  feed_step(retry_at+FEED_STALL_DURATION_MS+t,0,true);
  assert(outputs[9]==0 && setpoints[9]==0);
  assert(setpoints[6]==-SHOOTER_CONST_SPEED && setpoints[7]==SHOOTER_CONST_SPEED);
 }
 can_armed=false; shooter_command(); assert_shooter_stopped(); can_armed=true;
 feed_step(now_ms+1,0,true);
 assert(ShooterApp_GetController()->feed_recovery.state==FEED_RECOVERY_BLOCKED);
 assert(outputs[9]==0);
 feed_step(now_ms+1,0,false);
 assert(!ShooterApp_GetController()->feed_recovery.reverse_attempted);
 trigger_feed_reverse(now_ms+1,-g_feed_stall_current_threshold);
 feed_step(now_ms+1,INT16_MIN,false);
 assert(outputs[9]==0 && setpoints[9]==0);
 assert(ShooterApp_GetController()->feed_recovery.state==FEED_RECOVERY_MONITORING);
 assert(ShooterApp_GetController()->turntable_pid.output==0);

 /* 每个发射电机失联和CAN锁定都能中断反转，恢复不补发已用的一次。 */
 const uint8_t ids[]={6,7,9};
 for(unsigned failure=0;failure<4;++failure) {
  MsgCenter_Init(queue,32); ShooterApp_Init(); now_ms=10000;
  trigger_feed_reverse(now_ms,INT16_MIN);
  if(failure<3) {
   now_ms+=101;
   for(unsigned i=0;i<3;++i) if(i!=failure) shooter_feedback(ids[i],now_ms);
  } else can_armed=false;
  shooter_command(); assert_shooter_stopped();
  assert(ShooterApp_GetController()->feed_recovery.reverse_attempted);
  assert(ShooterApp_GetController()->feed_recovery.state!=FEED_RECOVERY_REVERSING);
  can_armed=true;
  feed_step(now_ms+1,0,true);
  assert(setpoints[9]==SHOOTER_RAMP_STEP);
 }

 /* 控制暂停超过反馈时限后，即使收到新帧，也不能把空档算成连续高电流。 */
 MsgCenter_Init(queue,32); ShooterApp_Init();
 feed_step(20000,INT16_MIN,true);
 feed_step(21000,INT16_MIN,true);
 assert(!ShooterApp_GetController()->feed_recovery.reverse_attempted);
 /* 不均匀回调频率按毫秒累计；同时间重复回调不提前触发。 */
 for(uint32_t t=21001;t<21500;t+=37) feed_step(t,INT16_MIN,true);
 for(unsigned i=0;i<1000;++i) shooter_command();
 assert(!ShooterApp_GetController()->feed_recovery.reverse_attempted);
 feed_step(21500,INT16_MIN,true);
 assert(outputs[9]<0);

 /* uint32回绕跨越高电流计时和反转计时；零时刻也可作为有效起点。 */
 const uint32_t starts[]={0,UINT32_MAX-250U,UINT32_MAX-FEED_STALL_DURATION_MS-50U};
 for(unsigned i=0;i<3;++i) {
  MsgCenter_Init(queue,32); ShooterApp_Init();
  trigger_feed_reverse(starts[i],INT16_MIN);
  for(uint32_t t=10;t<=FEED_REVERSE_DURATION_MS;t+=10)
   feed_step(starts[i]+FEED_STALL_DURATION_MS+t,INT16_MIN,true);
  assert(setpoints[9]==0 && outputs[9]==0);
 }
 /* 跳过Update直调计算入口，反转仍到期结束，失联仍归零。 */
 MsgCenter_Init(queue,32); ShooterApp_Init();
 trigger_feed_reverse(30000,INT16_MIN);
 ShooterController *controller=(ShooterController *)ShooterApp_GetController();
 now_ms+=FEED_REVERSE_DURATION_MS;
 for(unsigned i=0;i<3;++i)
  ShooterController_UpdateMotorFeedback(controller,ids[i],0,0,0,0,now_ms);
 ShooterController_ComputeCurrents(controller,now_ms);
 assert(outputs[9]==0 && setpoints[9]==0);
 now_ms+=101;
 ShooterController_ComputeCurrents(controller,now_ms);
 assert_shooter_stopped();
}

/* 自定义阈值用于正负反馈比较；无效阈值不触发反转或继续拨弹。 */
static void test_feed_custom_threshold(void) {
 MsgEvent queue[32];
 int16_t saved_threshold=g_feed_stall_current_threshold;
 MsgCenter_Init(queue,32); ShooterApp_Init();
 g_feed_stall_current_threshold=1600;
 for(uint32_t t=0;t<=1000;t+=10) feed_step(t,-1599,true);
 assert(!ShooterApp_GetController()->feed_recovery.reverse_attempted);
 trigger_feed_reverse(1010,-1600);
 feed_step(now_ms+1,0,false);
 g_feed_stall_current_threshold=400;
 trigger_feed_reverse(now_ms+1,400);
 const int16_t invalid[]={0,-1};
 for(unsigned i=0;i<2;++i) {
  feed_step(now_ms+1,0,false);
  g_feed_stall_current_threshold=invalid[i];
  feed_step(now_ms+1,0,true);
  assert(outputs[9]==0 && setpoints[9]==0);
  assert(ShooterApp_GetController()->feed_recovery.state==FEED_RECOVERY_BLOCKED);
  assert(!ShooterApp_GetController()->feed_recovery.reverse_attempted);
 }
 feed_step(now_ms+1,0,false);
 g_feed_stall_current_threshold=saved_threshold;
}

static void gimbal_step(uint32_t now,bool enabled,bool fresh){
 now_ms=now;
 if(fresh)motors[5].last_feedback_time=motors[8].last_feedback_time=now;
 GimbalCmd cmd={.enabled=enabled};
 assert(MsgCenter_Publish(TOPIC_GIMBAL_CMD,&cmd,sizeof(cmd))==0);
 /* 普通队列在控制命令发布后溢出，真实云台回调仍须本轮执行并保持失联保护。 */
 for (unsigned i=0;i<256U;i++) assert(MsgCenter_Publish(TOPIC_CAN_RX,&i,sizeof(i))==0);
 MsgCenter_Dispatch();
 assert(MsgCenter_GetDiagnostics()->overwritten_by_topic[TOPIC_GIMBAL_CMD]==0);
}

static void feedforward_step(GimbalCmd command) {
 now_ms += 4U;
 motors[5].last_feedback_time = motors[8].last_feedback_time = now_ms;
 assert(MsgCenter_Publish(TOPIC_GIMBAL_CMD, &command, sizeof(command)) == 0);
 MsgCenter_Dispatch();
}

/* Exercise actual controller callbacks with synthetic gains; production gains stay intact. */
static void test_gimbal_feedforward(MotorConfig_t *yaw_config, YawControlConfig *yaw_control) {
 MotorContext_t *yaw = &motors[5], *pitch = &motors[8];
 const MotorConfig_t *original_pitch = pitch->config;
 MotorConfig_t pitch_config = *original_pitch;
 PitchControlConfig pitch_control = *pitch_config.pitch_control;
 /* 固定4ms测试输入产生6ticks，单独验证前馈，不依赖实车摇杆倍率。 */
 pitch_control.manual_rate_deg_s = 659.1796875f;
 pitch_control.manual_stick_gain = 1.0f;
 pitch_control.load_gain=0; /* 前馈兼容测试隔离本次经验负载项。 */
 pitch_config.pitch_control = &pitch_control;
 pitch->config = &pitch_config;
 pitch_config.limits.gm6020.gravity_compensation = 0;
 /* 前馈测试锁存实测位置，独立验证负数初始目标的兼容路径。 */
 pitch_config.limits.gm6020.initial_angle = -1;
 yaw_config->feedforward = (GimbalFeedforwardConfig){10, 3, 100};
 pitch_config.feedforward = (GimbalFeedforwardConfig){4, -7, 100};
 yaw_control->speed_loop_only = true;
 yaw_control->manual_speed_rpm = 10;
 yaw->angle_raw = 4555; pitch->angle_raw = 1971;
 yaw->speed_rpm = pitch->speed_rpm = 0;
 PID_Init(&yaw->pid_inner, 0, 0, 0, 25000, 0);
 PID_Init(&pitch->pid_outer, 1, 0, 0, 600, 0);
 PID_Init(&pitch->pid_inner, 0, 0, 0, 25000, 0);
 pitch->pid_outer.output_lpf_rc = 0;
 gimbal_step(2200, true, true);
 assert(outputs[5] == 0 && outputs[8] == 0); /* Bias cannot bypass startup wait. */
 gimbal_step(2300, true, true);
 assert(outputs[5] == 0 && outputs[8] == -7); /* Yaw standstill suppresses bias; pitch retains it. */
 assert(yaw->pid_inner.last_time_us == now_ms * 1000U && yaw->pid_inner.dt == 0);
 GimbalCmd cmd = {.enabled = true, .yaw_rate = 1, .pitch_rate = -0.1f};
 PID_Reset(&pitch->pid_outer);
 feedforward_step(cmd);
 assert(yaw->pid_inner.target == 5 && outputs[5] == 53);
 assert(fabsf(yaw->pid_inner.dt - 0.004f) < 0.000001f);
 assert(pitch->pid_inner.target == 6 && outputs[8] == 17); /* direction already in speed target */
 assert(yaw->pid_outer.output == 0); /* Feedforward does not re-enable outer loop. */
 cmd.yaw_rate = -1; cmd.pitch_rate = 0.1f;
 pitch->angle_target = pitch->angle_raw; PID_Reset(&pitch->pid_outer);
 feedforward_step(cmd);
 assert(outputs[5] == -47 && outputs[8] == -31);
 yaw_config->feedforward.output_max = 20;
 pitch_config.feedforward.output_max = 8;
 feedforward_step(cmd);
 assert(outputs[5] == -20 && outputs[8] == -8);

 /* The feedforward uses the limited reference, then adds to the real PID output. */
 yaw_config->feedforward.output_max = 100;
 yaw_control->manual_speed_rpm = 3;
 cmd.yaw_rate = 1; cmd.pitch_rate = 0;
 feedforward_step(cmd);
 assert(yaw->pid_inner.target == 3 && outputs[5] == 33);
 PID_Init(&yaw->pid_inner, 2, 0, 0, 25000, 0);
 yaw->pid_inner.output_lpf_rc = 0;
 feedforward_step(cmd);
 assert(yaw->pid_inner.output == 6 && outputs[5] == 39);
 yaw_control->manual_speed_rpm = 10;
 PID_Init(&yaw->pid_inner, 0, 0, 0, 25000, 0);
 yaw_config->feedforward = (GimbalFeedforwardConfig){0, 50000, 60000};
 pitch_config.feedforward = yaw_config->feedforward;
 feedforward_step(cmd);
 assert(outputs[5] == MotorDriver_GetCommandLimit(5) && outputs[8] == 8000);
 yaw_config->feedforward.bias = pitch_config.feedforward.bias = -50000;
 feedforward_step(cmd);
 assert(outputs[5] == -MotorDriver_GetCommandLimit(5) && outputs[8] == -8000);

 /* Disabling the new term preserves the existing gravity compensation. */
 yaw_config->feedforward = (GimbalFeedforwardConfig){NAN, NAN, 0};
 pitch_config.feedforward = yaw_config->feedforward;
 pitch_config.limits.gm6020.gravity_compensation = 50;
 pitch_config.limits.gm6020.gravity_zero_angle = 0;
 pitch->angle_raw = 2048;
 feedforward_step(cmd);
 assert(outputs[5] == 0 && outputs[8] == -50);
 /* 重力零点独立于启动目标，零点两侧应给出反号补偿，不改变绝对限位。 */
 pitch_config.limits.gm6020.gravity_compensation = 5000;
 pitch_config.limits.gm6020.gravity_zero_angle = 1971;
 pitch->angle_raw = 1971;
 feedforward_step(cmd);
 assert(outputs[8] == 0);
 pitch->angle_raw = 1871;
 feedforward_step(cmd);
 assert(outputs[8] == 383);
 pitch->angle_raw = 2071;
 feedforward_step(cmd);
 assert(outputs[8] == -383);
 pitch_config.limits.gm6020.gravity_compensation = 0;
 yaw_config->feedforward = (GimbalFeedforwardConfig){0, 12, 100};
 pitch_config.feedforward = (GimbalFeedforwardConfig){0, -15, 100};
 gimbal_step(now_ms + 4, false, true);
 assert(outputs[5] == 0 && outputs[8] == 0);
 gimbal_step(now_ms + 4, true, true);
 gimbal_step(now_ms + 100, true, true);
 assert(outputs[5] == 0 && outputs[8] == -15);
 gimbal_step(now_ms + 101, true, false);
 assert(outputs[5] == 0 && outputs[8] == 0);
 gimbal_step(now_ms + 4, true, true);
 gimbal_step(now_ms + 100, true, true);
 assert(outputs[5] == 0 && outputs[8] == -15);

 /* Each invalid enabled term stops BOTH axes and clears their PID histories. */
 for (unsigned i = 0; i < 4; ++i) {
   if (i == 0) yaw_config->feedforward.velocity_gain = NAN;
   if (i == 1) pitch_config.feedforward.output_max = -1;
   if (i == 2) pitch_config.feedforward.bias = INFINITY;
   if (i == 3) yaw_config->feedforward.velocity_gain = FLT_MAX;
   if (i == 0) {
     gimbal_step(now_ms + 4, true, true); /* Invalid yaw feedforward also stops pitch at rest. */
     assert(outputs[5] == 0 && outputs[8] == 0);
   }
   feedforward_step(cmd);
   assert(outputs[5] == 0 && outputs[8] == 0);
   assert(yaw->pid_inner.output == 0 && pitch->pid_inner.output == 0);
   yaw_config->feedforward = (GimbalFeedforwardConfig){0, 12, 100};
   pitch_config.feedforward = (GimbalFeedforwardConfig){0, -15, 100};
   gimbal_step(now_ms + 4, true, true);
   assert(outputs[5] == 0 && outputs[8] == 0);
   gimbal_step(now_ms + 100, true, true);
   assert(outputs[5] == 0 && outputs[8] == -15);
 }
 pitch->config = original_pitch;
 yaw_config->feedforward = (GimbalFeedforwardConfig){0};
}
/* 关闭时yaw移动不能拉走最高位置目标；开启和自瞄覆盖分别验证。 */
static void test_pitch_coupling_switch(void) {
 MotorContext_t *pitch = &motors[8], *yaw = &motors[5];
 const MotorConfig_t *original = pitch->config;
 MotorConfig_t config = *original;
 SensorData sensor = {0};
 const float lower = config.limits.gm6020.angle_min;
 const float upper = config.limits.gm6020.angle_max;
 pitch->config = &config;
 config.limits.gm6020.enable_yaw_pitch_compensation = false;
 pitch->angle_raw = 2048; /* 原tan模型的奇异点也不能影响关闭后的目标。 */
 pitch->angle_target = lower + 4;
 now_ms += 4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8, 1, &sensor, false);
 assert(pitch->angle_target == lower);
 yaw->angle_raw -= 4;
 now_ms += 4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8, 0, &sensor, false);
 assert(pitch->angle_target == lower);

 config.limits.gm6020.enable_yaw_pitch_compensation = true;
 pitch->angle_raw = 1628;
 now_ms += 4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8, 0, &sensor, false);
 assert(pitch->angle_target == lower); /* 开关恢复不能补算已跳过的yaw转角。 */
 yaw->angle_raw -= 4;
 now_ms += 4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8, 0, &sensor, false);
 assert(pitch->angle_target > lower && pitch->angle_target < upper);
 float held = pitch->angle_target;
 yaw->angle_raw -= 4;
 now_ms += 4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8, 0, &sensor, true);
 assert(pitch->angle_target == held); /* 自瞄覆盖优先于配置开启。 */
 config.limits.gm6020.enable_yaw_pitch_compensation = false;
 yaw->angle_raw -= 4;
 now_ms += 4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8, 0, &sensor, false);
 assert(pitch->angle_target == held);
 pitch->config = original;
}

/* 控制器级验证：按时间积分而非按回调次数，倍率/单一阻尼配置实际接入。 */
static void test_pitch_timing_and_limits(void) {
 MotorContext_t *pitch=&motors[8];
 const MotorConfig_t *original=pitch->config;
 MotorConfig_t config=*original;
 PitchControlConfig control=*config.pitch_control;
 assert(control.load_gain==0 && PitchControl_Load(&control,2101)==0);
 config.pitch_control=&control;
 pitch->config=&config;
 pitch->angle_raw=1971; pitch->speed_rpm=0;
 control.manual_rate_deg_s=100; control.manual_stick_gain=0.5f;
 gimbal_step(now_ms+4,false,true);
 gimbal_step(now_ms+4,true,true);
 gimbal_step(now_ms+100,true,true);
 SensorData sensor={0};
 float start=pitch->angle_target;
 now_ms+=4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8,1,&sensor,true);
 float first=pitch->angle_target;
 assert(fabsf(first-(start-100*.5f*.004f*8192/360))<.001f);
 (void)GimbalController_PitchControl(8,1,&sensor,true);
 assert(pitch->angle_target==first); /* 重复毫秒不积分。 */
 now_ms+=8; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8,1,&sensor,true);
 assert(fabsf(pitch->angle_target-(start-100*.5f*.012f*8192/360))<.001f);
 float held=pitch->angle_target;
 now_ms+=4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8,0,&sensor,true);
 assert(pitch->angle_target==held);
 /* 无分段：大误差沿用外环上限，小误差不会被12RPM裁剪。 */
 PID_Init(&pitch->pid_outer,1,0,0,300,0);
 pitch->pid_outer.output_lpf_rc=0;
 pitch->angle_target=2300;
 for(unsigned i=0;i<6;i++) {
  now_ms+=4; pitch->last_feedback_time=now_ms;
  (void)GimbalController_PitchControl(8,0,&sensor,true);
 }
 assert(pitch->pid_inner.target==300);
 pitch->angle_target=2000; PID_Reset(&pitch->pid_outer);
 now_ms+=4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8,0,&sensor,true);
 assert(pitch->pid_inner.target==29);
 pitch->angle_target=1971; pitch->speed_rpm=60; PID_Reset(&pitch->pid_outer);
 now_ms+=4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8,0,&sensor,true);
 assert(pitch->pid_inner.target<0);
 /* 零阻尼时直接采用位置环输出，不再插入制动权重。 */
 control.velocity_damping_gain=0;
 PID_Reset(&pitch->pid_outer);
 now_ms+=4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8,0,&sensor,true);
 assert(pitch->pid_inner.target==0);
 /* 超时不累积大步长；下一次正常使能重新使用启动目标。 */
 held=pitch->angle_target;
 now_ms+=21; pitch->last_feedback_time=now_ms;
 assert(GimbalController_PitchControl(8,1,&sensor,true)==0);
 assert(pitch->angle_target==held);
 pitch->speed_rpm=0;
 gimbal_step(now_ms+4,true,true);
 gimbal_step(now_ms+100,true,true);
 assert(pitch->angle_target==1971);
 control.velocity_damping_gain=NAN;
 gimbal_step(now_ms+4,true,true);
 assert(outputs[5]==0 && outputs[8]==0);
 pitch->config=original;
 gimbal_step(now_ms+4,true,true);
 gimbal_step(now_ms+100,true,true);
 puts("pitch control: PASS (time integration, hold, unsegmented target, damping, timeout, invalid config)");
}

static void test_pitch_output_and_load(void) {
 MotorContext_t *pitch=&motors[8];
 const MotorConfig_t *original=pitch->config;
 MotorConfig_t config=*original;
 PitchControlConfig control=*config.pitch_control;
 assert(control.load_gain==0 && PitchControl_Load(&control,2101)==0);
 config.pitch_control=&control; pitch->config=&config;
 pitch->angle_raw=1971; pitch->speed_rpm=0;
 gimbal_step(now_ms+4,false,true);gimbal_step(now_ms+4,true,true);
 gimbal_step(now_ms+100,true,true);
 /* 接近端点时仍为原始PID+单一速度阻尼，不插入硬切断或软权重。 */
 PID_Init(&pitch->pid_outer,1,0,0,300,0);
 PID_Init(&pitch->pid_inner,100,0,0,8000,500);
 pitch->pid_outer.output_lpf_rc=0; pitch->pid_inner.output_lpf_rc=0;
 pitch->angle_raw=1651; pitch->angle_target=1607; pitch->speed_rpm=-25;
 now_ms+=4; pitch->last_feedback_time=now_ms;
 int16_t command=GimbalController_PitchControl(8,0,NULL,true);
 assert(fabsf(pitch->pid_inner.target-(-44+25*control.velocity_damping_gain))<.001f);
 assert(command==(int16_t)pitch->pid_inner.output && command<0);
 /* 速度过零/换向不抹掉负载积分；关闭其他项检验控制器真实路径。 */
 pitch->angle_raw=1971; pitch->angle_target=1971; pitch->speed_rpm=0;
 PID_Init(&pitch->pid_outer,0,0,0,300,0);
 PID_Init(&pitch->pid_inner,0,5,0,8000,2000);
 pitch->pid_inner.output_lpf_rc=0; pitch->pid_inner.iout=250;
 now_ms+=4; pitch->last_feedback_time=now_ms;
 command=GimbalController_PitchControl(8,0,NULL,true);
 assert(fabsf(pitch->pid_inner.iout-250)<.01f);
 assert(abs(command-(int)(250+PitchControl_Load(&control,1971)))<=1);
 /* 补偿随实际位置而非目标跳变；单独测试闭环总输出8000限幅。 */
 control.load_bias=20000;control.load_slope_per_tick=0;control.load_gain=1;
 control.load_output_max=20000;
 now_ms+=4; pitch->last_feedback_time=now_ms;
 command=GimbalController_PitchControl(8,0,NULL,true);
 assert(command==8000);
 /* 越界停机锁存：回弹到安全范围也不得自动重新追向端点。 */
 pitch->angle_raw=1606;gimbal_step(now_ms+4,true,true);
 assert(outputs[5]==0 && outputs[8]==0);
 pitch->angle_raw=1700;
 for(unsigned i=0;i<40;i++)gimbal_step(now_ms+4,true,true);
 assert(outputs[5]==0 && outputs[8]==0);
 pitch->config=original; pitch->angle_raw=1971;
 gimbal_step(now_ms+4,false,true);gimbal_step(now_ms+4,true,true);
 gimbal_step(now_ms+100,true,true);
 assert(pitch->angle_target==1971);
 puts("pitch limit/load: PASS (unscaled PID, residual I, total cap, latched fault)");
}

static void test_vision_targets(YawControlConfig *cfg) {
 cfg->speed_loop_only=false;
 MotorContext_t *yaw=&motors[5], *pitch=&motors[8];
 yaw->angle_raw=4000;pitch->angle_raw=1971;
 yaw->speed_rpm=pitch->speed_rpm=0;
 gimbal_step(now_ms+4,false,true);gimbal_step(now_ms+4,true,true);
 gimbal_step(now_ms+100,true,true);
 GimbalCmd cmd={.enabled=true,.vision_valid=true,.vision_frame=1,
   .vision_yaw_err_rad=.1f,.vision_pitch_err_rad=.1f,
   .yaw_rate=-1,.pitch_rate=-1};
 const float delta=.1f*8192.0f/(2.0f*(float)M_PI);
 feedforward_step(cmd);
 assert(fabsf(yaw->angle_target-(4000+delta))<.01f);
 assert(fabsf(pitch->angle_target-(1971+delta))<.01f);
 yaw->angle_raw+=10;pitch->angle_raw+=10;
 feedforward_step(cmd); /* Same frame: actual motion/joystick must not move target. */
 assert(fabsf(yaw->angle_target-(4000+delta))<.01f);
 assert(fabsf(pitch->angle_target-(1971+delta))<.01f);
 ++cmd.vision_frame;feedforward_step(cmd);
 assert(fabsf(yaw->angle_target-(4010+delta))<.01f);
 assert(fabsf(pitch->angle_target-(1981+delta))<.01f);
 ++cmd.vision_frame;cmd.vision_pitch_err_rad=1;feedforward_step(cmd);
 assert(pitch->angle_target==2374);
 ++cmd.vision_frame;cmd.vision_pitch_err_rad=-1;feedforward_step(cmd);
 assert(pitch->angle_target==1607);
 cmd.vision_valid=false;cmd.yaw_rate=cmd.pitch_rate=0;feedforward_step(cmd);
 assert(yaw->angle_target==yaw->angle_raw && pitch->angle_target==pitch->angle_raw);
 cmd.vision_valid=true;cmd.vision_pitch_err_rad=NAN;feedforward_step(cmd);
 assert(outputs[5]==0 && outputs[8]==0);
 puts("vision targets: PASS (both axes, one target per frame, limits, handover, invalid input)");
}

static void test_spin_shared_control(YawControlConfig *cfg) {
 YawControlConfig saved=*cfg;
 cfg->speed_loop_only=false;cfg->manual_speed_rpm=50;
 cfg->near_error_deg=8;cfg->approach_error_deg=55;
 cfg->near_speed_rpm=4;cfg->approach_speed_rpm=30;cfg->brake_speed_rpm=9;
 cfg->near_damping_gain=2;cfg->near_damping_blend_deg=0;
 cfg->near_brake_release_rpm=cfg->near_brake_full_rpm=0;
 MotorContext_t *yaw=&motors[5];
 yaw->angle_raw=4000;motors[8].angle_raw=1971;
 yaw->speed_rpm=100; /* 转子相对转速不可替代世界航向速度。 */
 gimbal_step(now_ms+4,false,true);gimbal_step(now_ms+4,true,true);
 gimbal_step(now_ms+100,true,true);
 PID_Init(&yaw->pid_outer,.5f,0,0,600,0);
 PID_Init(&yaw->pid_inner,2,0,0,12000,0);
 yaw->pid_outer.output_lpf_rc=yaw->pid_inner.output_lpf_rc=0;
 SensorData sensor={.yaw_total_angle=0,.g_gz=3*(float)M_PI/30};
 GimbalCmd cmd={.enabled=true,.yaw_rate_memo=1,.yaw_target_memo=1};
 MsgCenter_Publish(TOPIC_IMU_UPDATE,&sensor,sizeof(sensor));MsgCenter_Dispatch();
 for(unsigned i=0;i<10;i++)feedforward_step(cmd);
 assert(fabsf(yaw->pid_inner.actual-3)<.001f);
 assert(fabsf(yaw->pid_inner.target-4)<.001f); /* 共用近段上限。 */
 assert(g_gimbal_monitor.yaw_pid_kp==2);
 assert(fabsf(g_gimbal_monitor.yaw_pid_pout-2)<.001f);
 assert(g_gimbal_monitor.yaw.flags&GIMBAL_MONITOR_SPEED_IMU);
 sensor.g_gz=20*(float)M_PI/30;
 MsgCenter_Publish(TOPIC_IMU_UPDATE,&sensor,sizeof(sensor));MsgCenter_Dispatch();
 for(unsigned i=0;i<10;i++)feedforward_step(cmd);
 assert(fabsf(yaw->pid_inner.target+9)<.001f); /* 共用阻尼和全阶段反向制动。 */
 sensor.g_gz=NAN;
 MsgCenter_Publish(TOPIC_IMU_UPDATE,&sensor,sizeof(sensor));MsgCenter_Dispatch();
 feedforward_step(cmd);assert(outputs[5]==0 && outputs[8]==0);
 *cfg=saved;
 puts("spin shared control: PASS (motor PID, segmented limit, IMU damping, reverse cap, invalid feedback)");
}

int main(void){
 MsgEvent queue[32];
 MsgCenter_Init(queue,32);
 assert(MsgCenter_UseLatest(TOPIC_GIMBAL_CMD,MC_LATEST_CONTROL)==0);
 assert(MsgCenter_UseLatest(TOPIC_SHOOT_CMD,MC_LATEST_CONTROL)==0);
 for(uint8_t id=5;id<=8;id+=3){
  MotorContext_t *m=&motors[id];m->config=MotorService_GetConfig(id);m->initialized=true;
  m->type=MOTOR_TYPE_GM6020;m->role=m->config->role;m->angle_raw=id==5?5000:1900;
  PID_Init(&m->pid_outer,1,0,0,600,100);
  PID_Init(&m->pid_inner,1,0,0,25000,100);
 }
 /* 固定位置环测试路径，不让实车速度调试开关改变本测试的覆盖范围。 */
 MotorConfig_t yaw_config=*motors[5].config;
 YawControlConfig yaw_control=*yaw_config.yaw_control;
 yaw_control.speed_loop_only=false;
 yaw_control.manual_rate_deg_s=30; /* 固定测试输入，不依赖实车推杆灵敏度调参。 */
 yaw_config.yaw_control=&yaw_control;
 motors[5].config=&yaw_config;
 GimbalApp_Init(); ShooterApp_Init();
 gimbal_step(100,true,true);assert(outputs[5]==0&&outputs[8]==0);
 assert(g_gimbal_monitor.magic == 0x474D4F4E && g_gimbal_monitor.version == GIMBAL_MONITOR_VERSION);
 assert(g_gimbal_monitor.size_bytes == sizeof(GimbalMonitorSnapshot));
 assert(g_gimbal_monitor.callback_count == 1 && !(g_gimbal_monitor.sequence & 1));
 assert(!(g_gimbal_monitor.yaw.flags & GIMBAL_MONITOR_CONTROL_ACTIVE));
 gimbal_step(200,true,true);assert(motors[5].angle_target==5000);
 assert(motors[8].angle_target==1971); /* 初始目标不能被1900的反馈覆盖。 */
 motors[5].angle_raw=5100;
 for (unsigned n=0;n<PID_YAW_OUTER_DIVIDER;n++) gimbal_step(210+n,true,true);
 assert(outputs[5]!=0);
 assert(g_gimbal_monitor.yaw.command_raw == outputs[5]);
 assert(g_gimbal_monitor.pitch.command_raw == outputs[8]);
 assert(g_gimbal_monitor.yaw.flags & GIMBAL_MONITOR_POSITION_ACTIVE);
 assert(g_gimbal_monitor.yaw.command_unit == MOTOR_COMMAND_UNIT_CURRENT_COUNTS);
 assert(g_gimbal_monitor.pitch.command_unit == MOTOR_COMMAND_UNIT_VOLTAGE_COUNTS);
 assert(g_gimbal_monitor.pitch.position_target_ticks == motors[8].angle_target);
 assert(g_gimbal_monitor.yaw.speed_target_rpm == motors[5].pid_inner.target);
 motors[5].pid_inner.iout=999;
 gimbal_step(315,true,false);assert(outputs[5]==0&&outputs[8]==0&&motors[5].pid_inner.iout==0);
 assert(g_gimbal_monitor.yaw.command_raw == 0 && g_gimbal_monitor.pitch.command_raw == 0);
 assert(!(g_gimbal_monitor.yaw.flags & GIMBAL_MONITOR_CONTROL_ACTIVE));
 assert(g_gimbal_monitor.yaw.speed_target_rpm == 0);
 motors[5].angle_raw=1000;gimbal_step(320,true,true);assert(outputs[5]==0);
 gimbal_step(420,true,true);assert(motors[5].angle_target==1000&&outputs[5]==0);
 motors[5].pid_inner.iout=999;gimbal_step(421,false,true);
 assert(outputs[5]==0&&outputs[8]==0&&motors[5].pid_inner.iout==0);
 now_ms=500;
 for(uint8_t id=6;id<=9;id++){
  MotorFeedbackEvent f={.id=id,.speed=1000,.tick_ms=now_ms};
  MsgCenter_Publish(TOPIC_MOTOR_FEEDBACK,&f,sizeof(f));
 }
 MsgCenter_Dispatch();
 ShootCmd shoot={.friction_enabled=true,.feed_enabled=true};
 MsgCenter_Publish(TOPIC_SHOOT_CMD,&shoot,sizeof(shoot));MsgCenter_Dispatch();
 assert(setpoints[6]!=0&&setpoints[7]!=0);
 shoot=(ShootCmd){0};
 MsgCenter_Publish(TOPIC_SHOOT_CMD,&shoot,sizeof(shoot));MsgCenter_Dispatch();
 assert(outputs[6]<0&&outputs[7]<0&&outputs[9]==0); /* Fresh positive RPM requires braking at target zero. */
 assert(setpoints[6]==0&&setpoints[7]==0&&setpoints[9]==0);
 shoot.friction_enabled=true;
 for(unsigned i=0;i<15;i++) {
  MsgCenter_Publish(TOPIC_SHOOT_CMD,&shoot,sizeof(shoot));MsgCenter_Dispatch();
 }
 assert(setpoints[6]==-7500&&setpoints[7]==7500);
 shoot.friction_enabled=false;
 MsgCenter_Publish(TOPIC_SHOOT_CMD,&shoot,sizeof(shoot));MsgCenter_Dispatch();
 assert(setpoints[6]==-7000&&setpoints[7]==7000); /* Restore the original 500 RPM per callback ramp. */
 can_armed=false;
 MsgCenter_Publish(TOPIC_SHOOT_CMD,&shoot,sizeof(shoot));MsgCenter_Dispatch();
 assert(outputs[6]==0&&outputs[7]==0);
 assert(ShooterApp_GetController()->ramped_shooter1==0&&ShooterApp_GetController()->ramped_shooter2==0);
 can_armed=true;now_ms=601;
 MsgCenter_Publish(TOPIC_SHOOT_CMD,&shoot,sizeof(shoot));MsgCenter_Dispatch();
 assert(outputs[6]==0&&outputs[7]==0); /* Stale feedback must never be used to brake. */
 /* 固定测试增益，验证双环出力和失联归零，不把用户调参当作稳定性测试模型。 */
 MotorContext_t *yaw=&motors[5];
 PID_Init(&yaw->pid_outer,1,0,0,600,0);
 PID_Init(&yaw->pid_inner,1,0,0,25000,0);
 gimbal_step(700,true,true);gimbal_step(800,true,true);
 yaw->angle_raw+=20;
 for (unsigned n=0;n<2U*PID_YAW_OUTER_DIVIDER;n++) gimbal_step(810+n,true,true);
 assert(outputs[5]<0&&outputs[5]>=-MotorDriver_GetCommandLimit(5));
 yaw->angle_raw-=40;
 for (unsigned n=0;n<2U*PID_YAW_OUTER_DIVIDER;n++) gimbal_step(820+n,true,true);
 assert(outputs[5]>0&&outputs[5]<=MotorDriver_GetCommandLimit(5));
 motors[8].last_feedback_time=930;
 gimbal_step(930,true,false);
 assert(outputs[5]==0&&outputs[8]==0&&yaw->pid_inner.iout==0);
 /* 超出任一绝对边界均禁止启动双轴；进入范围后采用1971初始目标。 */
 MotorContext_t *pitch=&motors[8];
 const float lower=pitch->config->limits.gm6020.angle_min;
 const float upper=pitch->config->limits.gm6020.angle_max;
 pitch->angle_raw=lower-1;
 gimbal_step(1000,true,true);assert(outputs[5]==0&&outputs[8]==0);
 gimbal_step(1100,true,true);assert(outputs[5]==0&&outputs[8]==0);
 pitch->angle_raw=upper+1;
 gimbal_step(1104,true,true);assert(outputs[5]==0&&outputs[8]==0);
 pitch->angle_raw=lower;
 gimbal_step(1108,true,true);assert(pitch->angle_target==1971);
 SensorData sensor={0};
 pitch->angle_target=lower+4;
 for (unsigned i=0;i<10;i++) {
  now_ms+=4; pitch->last_feedback_time=now_ms;
  (void)GimbalController_PitchControl(8,1,&sensor,true);
 }
 assert(pitch->angle_target==lower); /* 连续抬头输入不能累积越过上止点。 */
 now_ms+=4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8,-1,&sensor,true);
 assert(fabsf(pitch->angle_target-(lower+32.768f))<0.001f); /* 边界仍允许反向离开。 */
 pitch->angle_raw=upper;pitch->angle_target=upper-5;
 for (unsigned i=0;i<10;i++) {
  now_ms+=4; pitch->last_feedback_time=now_ms;
  (void)GimbalController_PitchControl(8,-1,&sensor,true);
 }
 assert(pitch->angle_target==upper);
 now_ms+=4; pitch->last_feedback_time=now_ms;
 (void)GimbalController_PitchControl(8,1,&sensor,true);
 assert(fabsf(pitch->angle_target-(upper-32.768f))<0.001f);
 gimbal_step(1201,true,false);assert(outputs[5]==0&&outputs[8]==0);
 /* 配置中的启动目标也须在范围内，恢复合法配置后可在边界启动。 */
 MotorConfig_t limited_pitch=*pitch->config;
 limited_pitch.limits.gm6020.initial_angle=upper+1;
 pitch->config=&limited_pitch;
 gimbal_step(1300,true,true);gimbal_step(1400,true,true);
 assert(outputs[5]==0&&outputs[8]==0);
 pitch->config=MotorService_GetConfig(8);
 gimbal_step(1410,true,true);assert(pitch->angle_target==1971);
 /* 速度调试：位置/视觉/spin不能产生额外速度目标；回中仍主动制动。 */
 gimbal_step(1420,false,true);
 yaw_control.speed_loop_only=true;
 PID_Init(&yaw->pid_inner,100,0,0,25000,0);
 gimbal_step(1430,true,true);gimbal_step(1530,true,true);
 GimbalCmd speed_cmd={.enabled=true,.yaw_rate=1.0f,
   .trace={.flags=15,.rc_ch0=-660,.rc_sequence=41,.rc_dispatch_ms=1528,
           .route_sequence=79,.route_ms=1533}};
 now_ms=1534;motors[5].last_feedback_time=motors[8].last_feedback_time=now_ms;
 MsgCenter_Publish(TOPIC_GIMBAL_CMD,&speed_cmd,sizeof(speed_cmd));MsgCenter_Dispatch();
 assert(fabsf(yaw->pid_inner.target-5.0f)<0.001f&&outputs[5]>0);
 assert(g_gimbal_monitor.yaw.flags & GIMBAL_MONITOR_CONTROL_ACTIVE);
 assert(!(g_gimbal_monitor.yaw.flags & GIMBAL_MONITOR_POSITION_ACTIVE));
 assert(g_gimbal_monitor.yaw.speed_target_rpm == 5.0f);
 assert(g_gimbal_monitor.command_trace.rc_ch0 == -660);
 assert(g_gimbal_monitor.command_trace.rc_sequence == 41);
 assert(g_gimbal_monitor.command_trace.route_sequence == 79);
 assert(g_gimbal_monitor.yaw_route_rate == 1 && g_gimbal_monitor.yaw_mode == YAW_CONTROL_MANUAL);
 assert(g_gimbal_monitor.yaw_pid_pout == yaw->pid_inner.pout);
 assert(g_gimbal_monitor.yaw_pid_output == yaw->pid_inner.output);
 assert(g_gimbal_monitor.yaw_pid_kp == 100);
 assert(g_gimbal_monitor.yaw.command_raw == outputs[5]);
 assert(fabsf(g_gimbal_monitor.yaw.speed_loop_dt_s - 0.004f) < 0.000001f);
 assert(g_gimbal_monitor.callback_dt_ms == 4);
 /* 位置PID旁路后仍保留本次回调的角度参考，供遥测单独显示。 */
 assert(isfinite(g_gimbal_monitor.yaw.position_target_ticks));
 assert(g_gimbal_monitor.yaw.position_target_ticks == yaw->angle_target);
 assert(yaw->pid_outer.output==0.0f&&yaw->pid_outer.update_divider==0U);
 float held=yaw->angle_target;
 yaw->angle_raw+=100;yaw->speed_rpm=10;
 gimbal_step(1538,true,true);
 assert(yaw->pid_inner.target==0.0f&&outputs[5]<0&&yaw->angle_target==held);
 assert(g_gimbal_monitor.yaw.speed_target_rpm == 0.0f);
 assert(g_gimbal_monitor.command_trace.flags == 0); /* 非Cmd来源不继承上一命令的来源。 */
 assert(g_gimbal_monitor.yaw.position_target_ticks == held);
 speed_cmd.vision_valid=true;speed_cmd.vision_yaw_err_rad=1.0f;
 now_ms=1542;motors[5].last_feedback_time=motors[8].last_feedback_time=now_ms;
 MsgCenter_Publish(TOPIC_GIMBAL_CMD,&speed_cmd,sizeof(speed_cmd));MsgCenter_Dispatch();
 assert(yaw->pid_inner.target==0&&yaw->pid_inner.actual==10&&yaw->angle_target==held);
 speed_cmd.vision_valid=false;speed_cmd.yaw_rate_memo=1;speed_cmd.yaw_target_memo=90;
 now_ms=1546;motors[5].last_feedback_time=motors[8].last_feedback_time=now_ms;
 MsgCenter_Publish(TOPIC_GIMBAL_CMD,&speed_cmd,sizeof(speed_cmd));MsgCenter_Dispatch();
 assert(yaw->pid_inner.target==0&&yaw->pid_inner.actual==10);
 /* 最新帧虽新，但丢失超过20ms连续历史，不能猜中间圈数。 */
 gimbal_step(1570,true,true);assert(outputs[5]==0&&outputs[8]==0);
 yaw->speed_rpm=0;yaw->angle_raw=4555;
 gimbal_step(1574,true,true);gimbal_step(1674,true,true);
 assert(yaw->angle_target==4555&&yaw->pid_inner.target==0);
 /* 重新启用位置路径后，跨半圈仍追原目标，而不是改为顺向加速。 */
 gimbal_step(1678,false,true);yaw_control.speed_loop_only=false;
 yaw_control.manual_speed_rpm=10;yaw_control.vision_speed_rpm=7;
 gimbal_step(1682,true,true);gimbal_step(1782,true,true);
 for(unsigned k=1;k<=50;k++) {
   yaw->angle_raw=(uint16_t)((4555U+k*100U)%8192U);
   gimbal_step(1782+4*k,true,true);
 }
 assert(yaw->angle_target==4555&&yaw->pid_outer.error[0]<-4096);
 assert(yaw->pid_inner.target==-10&&outputs[5]<0);
 speed_cmd=(GimbalCmd){.enabled=true,.yaw_rate=1};
 now_ms=1986;motors[5].last_feedback_time=motors[8].last_feedback_time=now_ms;
 MsgCenter_Publish(TOPIC_GIMBAL_CMD,&speed_cmd,sizeof(speed_cmd));MsgCenter_Dispatch();
 assert(yaw->pid_inner.target==-10);
 gimbal_step(1990,true,true);assert(yaw->pid_inner.target==-10);
 /* 视觉与spin按明确模式限速，返回普通模式锁当前连续位置。 */
 speed_cmd.vision_valid=true;speed_cmd.vision_yaw_err_rad=1;
 now_ms=1994;motors[5].last_feedback_time=motors[8].last_feedback_time=now_ms;
 MsgCenter_Publish(TOPIC_GIMBAL_CMD,&speed_cmd,sizeof(speed_cmd));MsgCenter_Dispatch();
 assert(yaw->pid_inner.target==7);
 speed_cmd.vision_valid=false;speed_cmd.yaw_rate_memo=1;speed_cmd.yaw_target_memo=90;
 now_ms=1998;motors[5].last_feedback_time=motors[8].last_feedback_time=now_ms;
 MsgCenter_Publish(TOPIC_GIMBAL_CMD,&speed_cmd,sizeof(speed_cmd));MsgCenter_Dispatch();
 assert(g_gimbal_monitor.yaw.speed_target_rpm==10);
 /* spin世界航向超过半圈时仍保持所选圈数，不逐轮改走另一方向。 */
 sensor.yaw_total_angle=300;
 for(unsigned k=0;k<5;k++) {
   now_ms=2002+4*k;motors[5].last_feedback_time=motors[8].last_feedback_time=now_ms;
   MsgCenter_Publish(TOPIC_IMU_UPDATE,&sensor,sizeof(sensor));
   MsgCenter_Publish(TOPIC_GIMBAL_CMD,&speed_cmd,sizeof(speed_cmd));MsgCenter_Dispatch();
 }
 assert(g_gimbal_monitor.yaw.speed_target_rpm==-10);
 gimbal_step(2022,true,true);assert(yaw->angle_target==yaw->angle_raw&&
                                      g_gimbal_monitor.yaw.speed_target_rpm==0);
 assert(g_gimbal_monitor.yaw.speed_target_rpm == 0 && g_gimbal_monitor.yaw_pid_pout == 0);
 /* 配置缺失和非有限输入都归零两轴，不沿用旧命令。 */
 yaw_config.yaw_control=NULL;gimbal_step(2026,true,true);assert(outputs[5]==0&&outputs[8]==0);
 yaw_config.yaw_control=&yaw_control;
 gimbal_step(2030,true,true);gimbal_step(2130,true,true);
 speed_cmd=(GimbalCmd){.enabled=true,.yaw_rate=NAN};
 now_ms=2134;motors[5].last_feedback_time=motors[8].last_feedback_time=now_ms;
 MsgCenter_Publish(TOPIC_GIMBAL_CMD,&speed_cmd,sizeof(speed_cmd));MsgCenter_Dispatch();
 assert(outputs[5]==0&&outputs[8]==0);
 test_gimbal_feedforward(&yaw_config, &yaw_control);
 monitor_command_status = ROBOT_STATUS_UNSUPPORTED;
 gimbal_step(now_ms+4, true, true);
 assert(g_gimbal_monitor.yaw.command_status == ROBOT_STATUS_UNSUPPORTED);
 assert(!(g_gimbal_monitor.yaw.flags & GIMBAL_MONITOR_CONTROL_ACTIVE));
 monitor_command_status = ROBOT_STATUS_OK;
 test_pitch_timing_and_limits();
 test_pitch_coupling_switch();
 test_pitch_output_and_load();
 test_vision_targets(&yaw_control);
 test_spin_shared_control(&yaw_control);
 test_shooter_interlock();
 test_feed_stall_recovery();
 test_feed_custom_threshold();
 puts("control recovery: PASS (gimbal reseed, friction ramp/brake, feed stall/reverse, fault lock and stale feedback zero)");
}
