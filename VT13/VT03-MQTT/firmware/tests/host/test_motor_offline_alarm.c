/* 验证报警波形、故障轮询及时间边界；桩仅替换快照与物理输出。 */
#include "motor_offline_alarm.h"
#include "motor_service.h"
#include "bsp_alarm.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static MotorConfig_t configs[3];
static RobotConfig_t robot;
static MotorSnapshot snapshots[16];
static RobotStatus snapshot_status[16];
static bool red, green, blue, beep;
const RobotConfig_t *RobotConfig_Get(void) { return &robot; }
RobotStatus MotorService_GetSnapshot(uint8_t id, MotorSnapshot *result) {
 assert(id<16); *result=snapshots[id]; return snapshot_status[id];
}
void BspAlarm_Set(bool r, bool g, bool b, bool sound) {
 assert(!(r&&b)); assert(!(g&&(r||b||sound)));
 red=r; green=g; blue=b; beep=sound;
}
static void reset(void) {
 memset(configs,0,sizeof(configs)); memset(snapshots,0,sizeof(snapshots));
 memset(snapshot_status,0,sizeof(snapshot_status));
 configs[0]=(MotorConfig_t){.motor_id=0,.offline_alarm_id=6,.can_channel=CAN_CHANNEL_2};
 configs[1]=(MotorConfig_t){.motor_id=5,.offline_alarm_id=5,.can_channel=CAN_CHANNEL_1};
 configs[2]=(MotorConfig_t){.motor_id=8,.offline_alarm_id=4,.can_channel=CAN_CHANNEL_2};
 robot=(RobotConfig_t){.motor_configs=configs,.total_motor_count=3};
 for (unsigned i=0;i<16;i++) snapshots[i]=(MotorSnapshot){.initialized=true,.feedback_valid=true};
 MotorOfflineAlarm_Init(&robot);
 red=green=blue=beep=false;
}
static void step(uint32_t now) {
 /* 有效节点持续上报，失联节点用feedback_valid=false模拟。 */
 for (unsigned i=0;i<16;i++) if(snapshots[i].feedback_valid) snapshots[i].feedback_timestamp_ms=now;
 MotorOfflineAlarm_Task(now);
}
int main(void) {
 /* 全范围计数、每组后的完整蓝灯，以及同一故障的重复分组。 */
 for(unsigned id=1;id<=16;id++) for(unsigned bus=1;bus<=2;bus++) {
  reset(); configs[0].offline_alarm_id=id;
  configs[0].can_channel=bus==1?CAN_CHANNEL_1:CAN_CHANNEL_2;
  snapshots[0].feedback_valid=false;
  unsigned duration=id*200U>1000U?id*200U:1000U;
  for(unsigned group=0;group<2;group++) {
   unsigned reds=0,sounds=0; bool old_red=false,old_beep=false;
   for(unsigned t=0;t<duration+400U;t++) {
    step(group*(duration+400U)+t);
    if(red&&!old_red) reds++;
    if(beep&&!old_beep) sounds++;
    old_red=red;old_beep=beep;
    assert(!green);
    if(t>=duration) assert(blue&&!red&&!beep);
    else assert(!blue);
   }
   assert(reds==id && sounds==bus);
  }
 }
 reset(); snapshots[0].feedback_valid=false; snapshots[5].feedback_valid=false;
 for(unsigned t=0;t<1600;t++) step(t);
 assert(MotorOfflineAlarm_GetDiagnostics()->active_motor_id==0);
 step(1600); assert(MotorOfflineAlarm_GetDiagnostics()->active_motor_id==5);
 assert(MotorOfflineAlarm_GetDiagnostics()->offline_mask==((1U<<0)|(1U<<5)));
 /* 当前节点恢复，立即静音并蓝灯分隔，再选择剩余故障。 */
 snapshots[5].feedback_valid=true; step(1601); assert(blue&&!beep);
 for(unsigned t=1602;t<=2001;t++) step(t);
 assert(MotorOfflineAlarm_GetDiagnostics()->active_motor_id==0 && red);
 snapshots[0].feedback_valid=true;step(2002);assert(green&&!beep);
 assert(MotorOfflineAlarm_GetDiagnostics()->offline_mask==0);
 /* 年龄100ms边界、从未反馈、服务不可用、毫秒回绕。 */
 reset(); MotorOfflineAlarm_Task(100); assert(green);
 MotorOfflineAlarm_Task(101); assert(!green);
 assert(MotorOfflineAlarm_GetDiagnostics()->offline_mask==((1U<<0)|(1U<<5)|(1U<<8)));
 reset(); snapshots[8].initialized=false;step(0);assert(!green);
 reset(); snapshot_status[8]=ROBOT_STATUS_UNSUPPORTED;step(0);assert(!green);
 reset(); for(unsigned i=0;i<16;i++) snapshots[i].feedback_timestamp_ms=UINT32_MAX-50U;
 MotorOfflineAlarm_Task(20); assert(green);
 MotorOfflineAlarm_Task(50); assert(!green);
 reset(); snapshots[0].feedback_valid=false;
 for(unsigned t=0;t<=1600;t++) step(UINT32_MAX-700U+t);
 assert(red && !blue && MotorOfflineAlarm_GetDiagnostics()->active_motor_id==0);
 /* 卡顿不补发脉冲；重新经过蓝灯后才开始新组。 */
 reset(); snapshots[0].feedback_valid=false;step(0);step(500);assert(blue&&!beep);
 for(unsigned t=501;t<=900;t++) step(t);
 assert(red&&beep&&!blue);
 /* 缺少ID用常红代替猜码；非法车型明确暴露错误。 */
 reset();configs[0].offline_alarm_id=0;snapshots[0].feedback_valid=false;
 for(unsigned t=0;t<1000;t++){step(t);assert(red&&!blue);}
 step(1000);assert(blue);assert(MotorOfflineAlarm_GetDiagnostics()->unmapped_mask==1);
 reset();configs[0].motor_id=16;MotorOfflineAlarm_Init(&robot);step(0);
 assert(red&&!beep&&MotorOfflineAlarm_GetDiagnostics()->config_invalid);
 reset();configs[1].motor_id=0;MotorOfflineAlarm_Init(&robot);step(0);
 assert(MotorOfflineAlarm_GetDiagnostics()->config_invalid);
 puts("motor offline alarm: PASS (counts, blue separation, rotation, recovery, time wrap)");
}
