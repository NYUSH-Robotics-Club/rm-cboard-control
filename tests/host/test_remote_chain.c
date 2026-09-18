/* Run the copied USART registry, decoder and daemon through the real message bus.
 * Only HAL registers/time/logging are replaced; no hardware is driven. */
#include "main.h"
#include "bsp_rc.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "nyush_remote.h"
#include "message_center.h"
#include "cmd_controller.h"
#include "command_router.h"
#include "logger.h"
#include "motor_service.h"
#include "robot_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
TestUart test_uart;
static TestDma test_dma;
DMA_HandleTypeDef hdma_usart3_rx = { &test_dma };
UART_HandleTypeDef huart3 = { .Instance=&test_uart, .hdmarx=&hdma_usart3_rx };
static uint32_t now_ms;
static bool can_armed = true;
static bool can_ready = true;
static ChassisFollowConfig follow;
static RobotConfig_t robot;
static MotorSnapshot yaw_snapshot;
static RobotStatus yaw_status = ROBOT_STATUS_OK;
const RobotConfig_t *RobotConfig_Get(void) { return &robot; }
uint8_t MotorService_FindByRole(MotorRole_e role, uint8_t *ids, uint8_t count)
{ assert(role == MOTOR_ROLE_GIMBAL_YAW && count >= 1); ids[0] = 5; return 1; }
const MotorConfig_t *MotorService_GetConfig(uint8_t id)
{
    for (unsigned i = 0; i < robot.total_motor_count; ++i)
        if (robot.motor_configs[i].motor_id == id) return &robot.motor_configs[i];
    return NULL;
}
RobotStatus MotorService_GetSnapshot(uint8_t id, MotorSnapshot *snapshot)
{ assert(id == 5); *snapshot = yaw_snapshot; return yaw_status; }
void BspCan_Service(uint32_t now) { (void)now; }
bool BspCan_OutputsArmed(void) { return can_armed; }
bool BspCan_RecoveryReady(uint32_t now) { (void)now; return can_ready; }
bool BspCan_TryArm(uint32_t now) { (void)now; can_armed = can_ready; return can_armed; }
static HAL_UART_RxEventTypeTypeDef event_type;
static ChassisCmd chassis;
static GimbalCmd gimbal;
uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t BspTime_NowMs(void) { return now_ms; }
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *h)
{ assert(h == &huart3); return event_type; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *buffer, uint16_t size)
{
    assert(h == &huart3 && size == 18);
    if (test_dma.CR & DMA_SxCR_EN) return HAL_BUSY;
    h->pRxBuffPtr = buffer;
    h->ErrorCode = 0;
    test_dma.CR = DMA_SxCR_EN;
    test_dma.NDTR = size;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *b, uint16_t n, uint32_t timeout)
{ (void)h; (void)b; (void)n; (void)timeout; assert(0); return HAL_ERROR; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *b, uint16_t n)
{ return HAL_UART_Transmit(h,b,n,0); }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *b, uint16_t n)
{ return HAL_UART_Transmit(h,b,n,0); }
void Logger_Log(LogTag_t tag, LogLevel_t level, const char *fmt, ...)
{ (void)tag; (void)level; (void)fmt; }
void Logger_CSV(LogTag_t tag, const char *fmt, ...)
{ (void)tag; (void)fmt; }
static void on_command(const MsgEvent *event, void *user)
{
    (void)user;
    if (event->topic == TOPIC_CHASSIS_CMD) memcpy(&chassis,event->data,sizeof(chassis));
    else memcpy(&gimbal,event->data,sizeof(gimbal));
}
static void receive(const uint8_t *bytes, uint16_t size)
{
    memcpy(huart3.pRxBuffPtr, bytes, size);
    test_dma.CR = 0; /* HAL normal mode stops before invoking the event callback. */
    test_dma.NDTR = 18 - size;
    event_type = size == 18 ? HAL_UART_RXEVENT_TC : HAL_UART_RXEVENT_IDLE;
    BspRc_OnRxEvent(size);
    assert(test_dma.NDTR == 18 && (test_dma.CR & DMA_SxCR_EN));
    for (unsigned i=0;i<size;i++) assert(huart3.pRxBuffPtr[i] == 0);
}
int main(void)
{
    robot = g_robot_config_infantry_standard;
    assert(robot.chassis_follow && robot.chassis_follow->yaw_forward_ticks == 4555U);
    follow = *robot.chassis_follow;
    follow.yaw_ccw_sign = 1; /* 先测正向，再测反向；不依赖实车安装方向配置。 */
    robot.chassis_follow = &follow;
    MsgEvent queue[32];
    MsgCenter_Init(queue,32);
    CmdController_Init();
    remote_control_init();
    MsgCenter_Subscribe(TOPIC_CHASSIS_CMD,on_command,0);
    MsgCenter_Subscribe(TOPIC_GIMBAL_CMD,on_command,0);
    CmdController_Task(0);
    MsgCenter_Dispatch();
    assert(!chassis.enabled && !gimbal.enabled);
    const volatile BspRcDiagnostics *d = BspRc_GetDiagnostics();
    assert(d->start_attempts == 1);
    for (now_ms=10;now_ms<=100;now_ms+=10) MsgCenter_Dispatch();
    assert(d->start_attempts == 1);
    MsgCenter_Dispatch(); /* 110 ms: upstream offline callback retries, no abort. */
    assert(d->start_attempts == 2 && d->busy_starts == 1 && d->failed_starts == 0);
    now_ms=120; MsgCenter_Dispatch();
    assert(d->start_attempts == 3 && d->busy_starts == 2);
    uint8_t frame[18] = {0};
    uint64_t bits=1024ULL | (1024ULL<<11) | (1024ULL<<22) | (1354ULL<<33) | (2ULL<<44) | (2ULL<<46);
    for(unsigned i=0;i<6;i++) frame[i]=(uint8_t)(bits>>(8*i));
    frame[17]=4;
    receive(frame,7);
    assert(d->idle_events == 1 && RC_GetFrameCount() == 0);
    now_ms=140;
    receive(frame,18);
    MsgCenter_Dispatch();
    CmdController_Task(now_ms);
    MsgCenter_Dispatch();
    assert(RC_GetFrameCount() == 1 && chassis.enabled && gimbal.enabled);
    assert(gimbal.trace.flags == 15 && gimbal.trace.rc_sequence == 1);
    assert(gimbal.trace.rc_dispatch_ms == 140 && gimbal.trace.route_ms == 140);
    assert(fabsf(chassis.vx+0.5f)<0.0001f);
    uint8_t snapshot[18]; RC_GetLastFrame(snapshot);
    assert(memcmp(snapshot,frame,18)==0);
    frame[0]=0xFF; frame[1]|=7; frame[10]=0x66; frame[14]=0xA5; frame[15]=0x5A;
    receive(frame,18);
    const RC_ctrl_t *rc=get_remote_control_point();
    assert(rc->rc.ch[0] == 0 && rc->rc.ch[3] == 330 && rc->mouse.z == 0 && rc->key.v == 0x5AA5);
    MsgCenter_Dispatch(); CmdController_Task(150); MsgCenter_Dispatch();
    for(now_ms=150;now_ms<=260;now_ms+=10) MsgCenter_Dispatch();
    NyushRemoteSnapshot upstream; assert(NyushRemote_GetSnapshot(&upstream));
    assert(upstream.channels[3] == 0); /* Original daemon clears decoded state. */
    CmdController_Task(351); MsgCenter_Dispatch();
    assert(!chassis.enabled && !gimbal.enabled);
    assert(gimbal.trace.flags == (GIMBAL_TRACE_PRESENT | GIMBAL_TRACE_RC_SEEN | GIMBAL_TRACE_OUTPUTS_ARMED));
    test_dma.CR=0; huart3.ErrorCode=4; HAL_UART_ErrorCallback(&huart3);
    assert(d->uart_errors == 1 && d->last_uart_error == 4 && d->last_hal_status == HAL_OK);
    assert(test_dma.CR & DMA_SxCR_EN);
    now_ms=370; receive(frame,18); MsgCenter_Dispatch(); CmdController_Task(now_ms); MsgCenter_Dispatch();
    assert(chassis.enabled && RC_GetFrameCount()==3);
    can_armed = false;
    now_ms=380; receive(frame,18); MsgCenter_Dispatch(); CmdController_Task(now_ms); MsgCenter_Dispatch();
    assert(!chassis.enabled && !gimbal.enabled && !can_armed);
    memset(frame,0,sizeof(frame));
    bits=1024ULL | (1024ULL<<11) | (1024ULL<<22) | (1024ULL<<33) | (2ULL<<44) | (2ULL<<46);
    for(unsigned i=0;i<6;i++) frame[i]=(uint8_t)(bits>>(8*i));
    frame[17]=4;
    for(now_ms=400;now_ms<=900;now_ms+=10) {
        receive(frame,18); MsgCenter_Dispatch(); CmdController_Task(now_ms); MsgCenter_Dispatch();
        assert(!gimbal.enabled);
        if(now_ms<900) assert(!can_armed);
    }
    assert(can_armed);
    now_ms=920; receive(frame,18); MsgCenter_Dispatch(); CmdController_Task(now_ms); MsgCenter_Dispatch();
    assert(gimbal.enabled && !chassis.enabled);

    /* Real command callback: calibration -> current encoder -> routed message.
     * IMU defaults remain zero, so these checks cannot pass using IMU yaw instead.
     */
    RemoteControlMessage remote = {0};
    remote.rc.s[0] = RC_SW_DOWN;
    remote.rc.s[1] = RC_SW_MID;
    remote.rc.ch[3] = -660;
    yaw_snapshot.initialized = yaw_snapshot.feedback_valid = true;
    const float positions[] = {4555, 6603, 2507, 459, 8191, 0};
    for (unsigned i = 0; i < sizeof(positions)/sizeof(positions[0]); ++i) {
        now_ms += 4;
        yaw_snapshot.position = positions[i];
        yaw_snapshot.feedback_timestamp_ms = now_ms;
        MsgCenter_Publish(TOPIC_RC_UPDATE, &remote, sizeof(remote));
        MsgCenter_Dispatch(); CmdController_Task(now_ms); MsgCenter_Dispatch();
        float theta = (positions[i] - 4555) * (6.28318530718f / 8192);
        assert(chassis.enabled);
        assert(fabsf(chassis.vx - cosf(theta)) < 0.0001f);
        assert(fabsf(chassis.vy - sinf(theta)) < 0.0001f);
    }
    follow.yaw_ccw_sign = -1;
    yaw_snapshot.position = 6603;
    CmdController_Task(now_ms); MsgCenter_Dispatch();
    assert(chassis.enabled && fabsf(chassis.vy + 1) < 0.0001f);
    CmdController_Task(now_ms + 21); MsgCenter_Dispatch();
    assert(!chassis.enabled && chassis.vx == 0 && chassis.vy == 0 && chassis.wz == 0);
    yaw_snapshot.position = NAN;
    yaw_snapshot.feedback_timestamp_ms = now_ms;
    CmdController_Task(now_ms); MsgCenter_Dispatch();
    assert(!chassis.enabled);
    yaw_snapshot.position = 8192;
    CmdController_Task(now_ms); MsgCenter_Dispatch();
    assert(!chassis.enabled);
    yaw_snapshot.position = 4555;
    yaw_snapshot.feedback_valid = false;
    CmdController_Task(now_ms); MsgCenter_Dispatch();
    assert(!chassis.enabled);
    yaw_snapshot.feedback_valid = true;
    yaw_status = ROBOT_STATUS_NOT_READY;
    CmdController_Task(now_ms); MsgCenter_Dispatch();
    assert(!chassis.enabled);
    yaw_status = ROBOT_STATUS_OK;
    follow.yaw_ccw_sign = 0;
    CmdController_Task(now_ms); MsgCenter_Dispatch();
    assert(!chassis.enabled);
    follow.yaw_ccw_sign = 1;
    CmdController_Task(now_ms); MsgCenter_Dispatch();
    assert(chassis.enabled && fabsf(chassis.vx - 1) < 0.0001f);
    /* 新RC可先于已发布命令派发；该命令必须保留路由时的来源，不被新RC覆盖。 */
    remote.rc.s[1] = RC_SW_DOWN;
    remote.rc.ch[0] = -330;
    now_ms += 4;
    MsgCenter_Publish(TOPIC_RC_UPDATE, &remote, sizeof(remote));
    MsgCenter_Dispatch();
    CmdController_Task(now_ms);
    uint32_t first_route_ms = now_ms;
    remote.rc.ch[0] = 330;
    now_ms += 4;
    MsgCenter_Publish(TOPIC_RC_UPDATE, &remote, sizeof(remote));
    MsgCenter_Dispatch();
    assert(gimbal.trace.rc_ch0 == -330 && gimbal.yaw_rate == 0.5f);
    assert(gimbal.trace.route_ms == first_route_ms && gimbal.trace.rc_dispatch_ms == first_route_ms);
    uint32_t old_rc_seq = gimbal.trace.rc_sequence, old_route_seq = gimbal.trace.route_sequence;
    CmdController_Task(now_ms); MsgCenter_Dispatch();
    assert(gimbal.trace.rc_ch0 == 330 && gimbal.yaw_rate == -0.5f);
    assert(gimbal.trace.rc_sequence == old_rc_seq + 1 && gimbal.trace.route_sequence == old_route_seq + 1);
    assert(gimbal.trace.rc_dispatch_ms == now_ms && gimbal.trace.route_ms == now_ms);
    now_ms += 4; CmdController_Task(now_ms); MsgCenter_Dispatch();
    assert(gimbal.trace.rc_sequence == old_rc_seq + 1 && gimbal.trace.route_sequence == old_route_seq + 2);
    puts("nyush remote chain: PASS (original registry/decoder/daemon, source provenance, TC/IDLE, snapshot, BUSY retry without abort, command mapping, loss, error and reconnection)");
}
