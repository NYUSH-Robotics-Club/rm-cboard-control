/*
 * Exercise message -> omni strategy -> speed/current PID -> CAN1 0x200 packing.
 * The motor-service shim forwards to real CAN aggregation; time/log/BSP are fake.
 * Geometry is synthetic and never installed on the real board.
 */
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "chassis_controller.h"
#include "chassis_strategy.h"
#include "robot_config.h"
#include "motor_service.h"
#include "control_messages.h"
#include "message_center.h"
#include "logger.h"
#include "can_manager.h"
#include "bsp_can.h"

CAN_Manager_t can1_manager, can2_manager;
static CAN_HandleTypeDef handles[2];
static MotorRegistry_t registry;
static RobotConfig_t robot;
static MotorConfig_t test_motors[16];
static OmniChassisConfig geometry;
static uint32_t now;
static float commanded_rpm[16];
static int16_t commanded_current[16];
static uint8_t last_frame[8];
static unsigned writes;
static MsgEvent queue[32];
const RobotConfig_t *RobotConfig_Get(void) { return &robot; }
uint32_t BspTime_NowMs(void) { return now; }
uint32_t BspTime_NowUs(void) { return now*1000U; }
void Logger_Log(LogTag_t t, LogLevel_t l, const char *f, ...) {(void)t;(void)l;(void)f;}
bool BspCan_Start(BspCanChannel c) {(void)c;return true;}
bool BspCan_OutputsArmed(void) {return true;}
bool BspCan_Read(BspCanChannel c, BspCanFrame *f) {(void)c;(void)f;return false;}
bool BspCan_MatchesNativeHandle(BspCanChannel c, const void *h)
{
    return c==BSP_CAN_CHANNEL_1 && h==&handles[0];
}
bool BspCan_Write(BspCanChannel c, uint16_t id, const uint8_t *d, uint8_t n)
{
    assert(c==BSP_CAN_CHANNEL_1 && id==0x200 && n==8);
    memcpy(last_frame,d,8); ++writes; return true;
}
const MotorConfig_t *MotorService_GetConfig(uint8_t id)
{
    for (uint8_t i=0;i<robot.total_motor_count;++i)
        if (robot.motor_configs[i].motor_id==id) return &robot.motor_configs[i];
    return NULL;
}
uint8_t MotorService_FindByRole(MotorRole_e role, uint8_t *ids, uint8_t cap)
{
    uint8_t count=0;
    for (uint8_t i=0;i<robot.total_motor_count && count<cap;++i)
        if (robot.motor_configs[i].role==role) ids[count++]=robot.motor_configs[i].motor_id;
    return count;
}
RobotStatus MotorService_CommandCurrent(uint8_t id, int16_t value)
{
    assert(id>=1 && id<=4); /* A bad geometry must never stop yaw or another role. */
    commanded_current[id]=value;
    assert(CAN_Manager_SendMotorCurrent(&can1_manager,id,value)==HAL_OK);
    return ROBOT_STATUS_OK;
}
RobotStatus MotorService_CommandConfigured(uint8_t id, float rpm, int16_t value)
{
    commanded_rpm[id]=rpm;
    return MotorService_CommandCurrent(id,value);
}
static void setup(void)
{
    const float c=.70710678118655f;
    robot=g_robot_config_infantry_standard;
    assert(robot.total_motor_count <= 16);
    memcpy(test_motors, robot.motor_configs, robot.total_motor_count * sizeof(test_motors[0]));
    robot.motor_configs = test_motors;
    /* Verify controller/slot behavior independently of the user's live PID tuning. */
    for (uint8_t i=0; i<robot.total_motor_count; ++i) {
        if (test_motors[i].role == MOTOR_ROLE_CHASSIS_DRIVE) {
            test_motors[i].pid_outer.kp = 10.0f;
            test_motors[i].pid_outer.ki = 0.0f;
            test_motors[i].pid_outer.kd = 0.0f;
            test_motors[i].pid_outer.output_max = 12000.0f;
            test_motors[i].pid_inner = (PIDParams_t){0.5f, 0.0f, 0.0f, 15000.0f, 3000.0f};
        }
    }
    geometry=(OmniChassisConfig){
        .configured=1,.max_translation_mps=1,.max_rotation_radps=1,
        .wheels={
            {3,.2f,.3f,c,-c,.05f,10},
            {2,.2f,-.3f,c,c,.05f,10},
            {1,-.2f,-.3f,c,-c,.05f,10},
            {4,-.2f,.3f,c,c,.05f,10}
        }
    };
    robot.omni=&geometry;
    MsgCenter_Init(queue,32);
    assert(CAN_Manager_Init(&can1_manager,CAN_CHANNEL_1,&handles[0],&robot,&registry)==HAL_OK);
    ChassisApp_Init();
    memset(commanded_rpm,0,sizeof(commanded_rpm));
    memset(commanded_current,0,sizeof(commanded_current));
    now=0; writes=0;
}
static void feedback(uint8_t id, uint32_t tick)
{
    MotorFeedbackEvent m={.id=id,.speed=0,.tick_ms=tick};
    assert(MsgCenter_Publish(TOPIC_MOTOR_FEEDBACK,&m,sizeof(m))==0);
}
static void command(ChassisCmd cmd)
{
    assert(MsgCenter_Publish(TOPIC_CHASSIS_CMD,&cmd,sizeof(cmd))==0);
    MsgCenter_Dispatch();
    assert(CAN_Manager_FlushTx(&can1_manager)==HAL_OK);
    assert(writes>0);
    /* ID1/2/3/4 always occupy bytes 0/2/4/6 regardless of geometry order. */
    for (uint8_t id=1;id<=4;++id) {
        unsigned slot=(id-1)*2;
        int16_t packed=(int16_t)((uint16_t)last_frame[slot]<<8 | last_frame[slot+1]);
        assert(packed==commanded_current[id]);
    }
}
static void expect_stopped(void)
{
    for (uint8_t id=1;id<=4;++id) assert(commanded_current[id]==0);
}
/* 零速目标配非零电流反馈，区分真实电流内环与仅替换速度PID数值。
 * 测试关闭输出滤波以精确核对原始电流刻度及两级限幅。
 */
static void test_current_loop(void)
{
    setup();
    ChassisController ctrl;
    ChassisController_Init(&ctrl);
    const float zero[4]={0};
    ChassisController_SetTargetSpeeds(&ctrl,zero);
    for (uint8_t i=0;i<4;++i) {
        assert(ctrl.current_loop_enabled[i]);
        ctrl.speed_pids[i].output_lpf_rc=0;
        ctrl.current_pids[i].output_lpf_rc=0;
        ChassisController_UpdateMotorFeedback(&ctrl,i,0,0,i%2?-1000:1000,25,1);
    }
    now=1;ChassisController_ComputeCurrents(&ctrl,now);
    for (unsigned i=0;i<4;++i) {
        float expected=i%2?500.0f:-500.0f;
        assert(fabsf(ctrl.current_pids[i].output-expected)<0.001f);
        /* 浮点滤波计算后转int16_t会向零截断，容许一刻度量化误差。 */
        assert(fabsf((float)ctrl.output_currents[i]-expected)<=1.0f);
    }
    const float fast[4]={10000,10000,10000,10000};
    ChassisController_SetTargetSpeeds(&ctrl,fast);
    for (uint8_t i=0;i<4;++i)
        ChassisController_UpdateMotorFeedback(&ctrl,i,0,0,-30000,25,2);
    now=2;ChassisController_ComputeCurrents(&ctrl,now);
    for (unsigned i=0;i<4;++i) {
        assert(ctrl.speed_pids[i].output==12000);
        assert(ctrl.output_currents[i]==15000);
        ctrl.speed_pids[i].iout=100;
        ctrl.current_pids[i].iout=100;
    }
    for (uint8_t i=1;i<4;++i)
        ChassisController_UpdateMotorFeedback(&ctrl,i,0,0,0,25,103);
    now=103;ChassisController_ComputeCurrents(&ctrl,now);
    expect_stopped();
    for (unsigned i=0;i<4;++i)
        assert(ctrl.speed_pids[i].iout==0 && ctrl.current_pids[i].iout==0);

    setup();
    for (unsigned i=0;i<robot.total_motor_count;++i)
        if (test_motors[i].role==MOTOR_ROLE_CHASSIS_DRIVE)
            test_motors[i].pid_inner=(PIDParams_t){0};
    ChassisController_Init(&ctrl);
    ChassisController_SetTargetSpeeds(&ctrl,zero);
    for (uint8_t i=0;i<4;++i) {
        assert(!ctrl.current_loop_enabled[i]);
        ChassisController_UpdateMotorFeedback(&ctrl,i,0,0,1000,25,1);
    }
    now=1;ChassisController_ComputeCurrents(&ctrl,now);expect_stopped();

    setup();test_motors[0].pid_inner.output_max=NAN;
    ChassisController_Init(&ctrl);ChassisController_SetTargetSpeeds(&ctrl,fast);
    for (uint8_t i=0;i<4;++i)
        ChassisController_UpdateMotorFeedback(&ctrl,i,0,0,0,25,1);
    now=1;ChassisController_ComputeCurrents(&ctrl,now);expect_stopped();
}
int main(void)
{
    test_current_loop();
    setup();
    ChassisCmd cmd={.vx=.3f,.vy=-.4f,.wz=.5f,.enabled=true};
    command(cmd); expect_stopped(); /* No feedback, even in the first 100 ms. */
    feedback(3,0); feedback(2,0); feedback(1,0);
    command(cmd); expect_stopped(); /* Three wheels are insufficient. */
    feedback(4,0); now=1;
    command(cmd);
    const float n=(1.0f/sqrtf(2.0f))/(3.14159265358979f*.1f)*60*10;
    assert(fabsf(commanded_rpm[3]-.45f*n)<.002f);
    assert(fabsf(commanded_rpm[2]-.15f*n)<.002f);
    assert(fabsf(commanded_rpm[1]+.95f*n)<.002f); /* ID1 direction=-1. */
    assert(fabsf(commanded_rpm[4]-.35f*n)<.002f); /* ID4 direction=-1. */
    assert(commanded_current[3]>0 && commanded_current[2]>0);
    assert(commanded_current[1]<0 && commanded_current[4]>0);

    now=101; feedback(3,101); feedback(2,101); feedback(1,101);
    command(cmd); expect_stopped(); /* One stale wheel stops all four. */
    feedback(4,101); now=102;
    command(cmd); assert(commanded_current[1]<0);
    cmd.enabled=false; now=103;
    command(cmd); expect_stopped(); /* Nonzero stale setpoints cannot bypass enable. */
    cmd.enabled=true; cmd.vx=NAN;
    command(cmd); expect_stopped();

    setup(); geometry.configured=0; MsgCenter_Init(queue,32); ChassisApp_Init();
    for (uint8_t id=1;id<=4;++id) feedback(id,0);
    cmd=(ChassisCmd){.vx=1,.enabled=true};
    command(cmd); expect_stopped();

    setup(); geometry.wheels[0].motor_id=5; MsgCenter_Init(queue,32); ChassisApp_Init();
    for (uint8_t id=1;id<=4;++id) feedback(id,0);
    command(cmd); expect_stopped(); /* Bad mapping does not touch yaw ID5. */

    setup(); geometry.wheels[0].radius_m=0;
    for (uint8_t id=1;id<=4;++id) feedback(id,0);
    command(cmd); expect_stopped();
    /* Actual user dimensions, using the production config rather than fixture.
     * 几何基准：0.3 m/s 对应555.72 RPM，0.6 rad/s 对应600.18 RPM；按当前限速缩放。
     */
    setup(); geometry=*g_robot_config_infantry_standard.omni;
    assert(geometry.configured);
    MsgCenter_Init(queue,32); ChassisApp_Init();
    for (uint8_t id=1;id<=4;++id) feedback(id,0);
    now=1;
    command((ChassisCmd){.vx=1,.enabled=true});
    for (uint8_t id=1;id<=4;++id)
        assert(fabsf(fabsf(commanded_rpm[id])-555.72f*geometry.max_translation_mps/.3f)<.04f);
    command((ChassisCmd){.wz=1,.enabled=true});
    /* 当前轮序2/1/4/3和安装方向下，正旋转四个转子目标均为负。 */
    const float rotation_by_id[4]={-600.18f,-600.18f,-600.18f,-600.18f};
    for (uint8_t id=1;id<=4;++id)
        assert(fabsf(commanded_rpm[id]-rotation_by_id[id-1]*geometry.max_rotation_radps/.6f)<.05f);
    puts("omni controller and CAN1 wheel slot integration: PASS");
    return 0;
}

void BspCan_TraceConfigure(BspCanChannel c, uint16_t tx, uint16_t rx, uint8_t slot)
{ (void)c; (void)tx; (void)rx; (void)slot; }
void BspCan_TraceCommand(BspCanChannel c, uint16_t tx, uint8_t slot, int16_t raw)
{ (void)c; (void)tx; (void)slot; (void)raw; }

void BspCan_TraceFeedback(BspCanChannel c, uint16_t id, uint32_t ms, int32_t raw, uint32_t detail)
{ (void)c; (void)id; (void)ms; (void)raw; (void)detail; }

BspCanTxResult BspCan_TryWrite(BspCanChannel c, uint16_t id, const uint8_t *d, uint8_t n)
{ return BspCan_Write(c,id,d,n) ? BSP_CAN_TX_ACCEPTED : BSP_CAN_TX_ERROR; }
