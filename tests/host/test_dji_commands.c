/* Exercise the real DJI adapter, registry, and CAN aggregation with captured BSP writes. */
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "can_manager.h"
#include "bsp_can.h"
#include "bsp_time.h"
#include "motor_adapter.h"
#include "motor_driver.h"
#include "dji_motor_protocol.h"
#include "robot_config.h"
#include "message_center.h"

CAN_Manager_t can1_manager, can2_manager;
static CAN_HandleTypeDef handles[2];
static MotorRegistry_t registries[2];
static MotorContext_t contexts[16];
static unsigned count;
static BspCanTxResult submit_result = BSP_CAN_TX_ACCEPTED;
static bool armed = true, disarm_on_submit;
static struct { BspCanChannel bus; uint16_t id; uint8_t data[8]; } frames[32];
static BspCanFrame received_frame;
static bool frame_ready;
static unsigned received_raw, received_decoded, received_commands;
static uint16_t received_ids;
static uint32_t receive_tick = 100;
uint32_t BspTime_NowMs(void) { return receive_tick; }
uint32_t BspTime_NowUs(void) { return 100000; }
bool BspCan_Start(BspCanChannel c) { (void)c; return true; }
bool BspCan_OutputsArmed(void) { return armed; }
bool BspCan_Read(BspCanChannel c, BspCanFrame *f) {
    (void)c;
    if (!frame_ready) return false;
    *f = received_frame; frame_ready = false; return true;
}
bool BspCan_MatchesNativeHandle(BspCanChannel c, const void *h)
{
    return (c == BSP_CAN_CHANNEL_1 && h == &handles[0]) ||
           (c == BSP_CAN_CHANNEL_2 && h == &handles[1]);
}
bool BspCan_Write(BspCanChannel c, uint16_t id, const uint8_t *data, uint8_t len)
{
    assert(len == 8 && count < 32);
    frames[count].bus = c; frames[count].id = id;
    memcpy(frames[count++].data, data, 8);
    return true;
}
/* The driver shim provides contexts and routes commands; the adapter and CAN code are real. */
MotorContext_t *MotorDriver_GetContext(uint8_t id) { return id < 16 ? &contexts[id] : NULL; }
void MotorDriver_SendCurrent(uint8_t id, int16_t value)
{
    CAN_Manager_t *m = contexts[id].config->can_channel == CAN_CHANNEL_1 ? &can1_manager : &can2_manager;
    assert(CAN_Manager_SendMotorCurrent(m, id, value) == HAL_OK);
}
void MotorDriver_FlushAll(void) { assert(CAN_Manager_FlushTx(&can1_manager) == HAL_OK); assert(CAN_Manager_FlushTx(&can2_manager) == HAL_OK); }
void MotorDriver_ResetPID(uint8_t id) { (void)id; }
int16_t MotorDriver_ComputeCurrent(uint8_t id, float value, bool angle) { (void)id; (void)value; (void)angle; return 0; }

static void expect(unsigned i, BspCanChannel c, uint16_t id, unsigned slot, int16_t value)
{
    uint8_t data[8] = {0};
    data[slot * 2] = (uint8_t)((uint16_t)value >> 8);
    data[slot * 2 + 1] = (uint8_t)value;
    assert(frames[i].bus == c && frames[i].id == id);
    assert(memcmp(frames[i].data, data, 8) == 0);
}

/* 用真实 CAN 解析和消息中心验证反馈洪水，不模拟控制算法或物理运动。 */
static void decoded_feedback(const MsgEvent *ev, void *user)
{
    (void)user;
    uint8_t id;
    if (ev->topic == TOPIC_GM6020_FEEDBACK) {
        const GM6020FeedbackEvent *f = (const GM6020FeedbackEvent *)ev->data;
        assert(f->angle == 999 && f->speed == -123 && f->current == -321);
        assert(f->tick_ms == 999);
        id = f->id;
    } else {
        const MotorFeedbackEvent *f = (const MotorFeedbackEvent *)ev->data;
        assert(f->angle == 999 && f->speed == -123 && f->current == -321);
        assert(f->tick_ms == 999);
        id = f->id;
    }
    received_ids |= (uint16_t)(1U << id);
    ++received_decoded;
}
static void raw_feedback(const MsgEvent *ev, void *user)
{
    (void)user;
    assert(ev->size == sizeof(CanRxFrame));
    ++received_raw;
}
static void receive_command(const MsgEvent *ev, void *user)
{
    (void)user;
    assert(received_decoded == RobotConfig_Get()->total_motor_count);
    assert(*(const uint32_t *)ev->data == 0U); /* 停机命令不能被反馈覆盖。 */
    ++received_commands;
}
static void test_receive_overload(const RobotConfig_t *robot)
{
    MsgEvent queue[16];
    MsgCenter_Init(queue, 16);
    assert(MsgCenter_UseLatest(TOPIC_GIMBAL_CMD, MC_LATEST_CONTROL) == 0);
    MsgCenter_Subscribe(TOPIC_MOTOR_FEEDBACK, decoded_feedback, NULL);
    MsgCenter_Subscribe(TOPIC_GM6020_FEEDBACK, decoded_feedback, NULL);
    MsgCenter_Subscribe(TOPIC_CAN_RX, raw_feedback, NULL);
    MsgCenter_Subscribe(TOPIC_GIMBAL_CMD, receive_command, NULL);
    uint32_t stop = 0;
    MsgCenter_Publish(TOPIC_GIMBAL_CMD, &stop, sizeof(stop));
    uint16_t expected_ids = 0;
    for (unsigned n = 0; n < 1000; ++n) {
        receive_tick = n;
        for (unsigned j = 0; j < robot->total_motor_count; ++j) {
            const MotorConfig_t *cfg = &robot->motor_configs[j];
            received_frame = (BspCanFrame){.standard_id=cfg->can_rx_id, .length=8,
                .is_standard_frame=true, .is_data_frame=true,
                .data={(uint8_t)(n >> 8), (uint8_t)n, 0xff, 0x85, 0xfe, 0xbf, 40, 0}};
            frame_ready = true;
            CAN_Manager_t *manager = cfg->can_channel == CAN_CHANNEL_1 ? &can1_manager : &can2_manager;
            CAN_Manager_ProcessCallback(manager, manager->hcan);
            expected_ids |= (uint16_t)(1U << cfg->motor_id);
        }
    }
    MsgCenter_Dispatch();
    assert(received_decoded == robot->total_motor_count && received_ids == expected_ids);
    assert(received_commands == 1 && received_raw == 0);
    assert(MsgCenter_GetDiagnostics()->overwritten == 0);
    assert(MsgCenter_GetDiagnostics()->published_by_topic[TOPIC_CAN_RX] == 0);
    MsgCenter_Dispatch(); assert(received_decoded == robot->total_motor_count);

    /* 非 DJI 帧和未知 ID 仍交给原始帧适配器，不根据测试数据猜解码布局。 */
    MotorConfig_t other = robot->motor_configs[0];
    other.vendor = MOTOR_VENDOR_DM; other.can_channel = CAN_CHANNEL_1;
    RobotConfig_t one = {.motor_configs=&other, .total_motor_count=1};
    assert(CAN_Manager_Init(&can1_manager, CAN_CHANNEL_1, &handles[0], &one, &registries[0]) == HAL_OK);
    received_frame.standard_id = other.can_rx_id; frame_ready = true;
    CAN_Manager_ProcessCallback(&can1_manager, &handles[0]);
    MsgCenter_Dispatch(); assert(received_raw == 1);
    received_frame.standard_id = 0x700; frame_ready = true;
    CAN_Manager_ProcessCallback(&can1_manager, &handles[0]);
    MsgCenter_Dispatch(); assert(received_raw == 2);
    assert(CAN_Manager_Init(&can1_manager, CAN_CHANNEL_1, &handles[0], robot, &registries[0]) == HAL_OK);
}

static void test_latest_pending(const RobotConfig_t *robot) {
 assert(CAN_Manager_Init(&can1_manager,CAN_CHANNEL_1,&handles[0],robot,&registries[0])==HAL_OK);
 count=0; submit_result=BSP_CAN_TX_BUSY;
 assert(CAN_Manager_SendMotorCurrent(&can1_manager,5,100)==HAL_OK);
 assert(CAN_Manager_FlushTx(&can1_manager)==HAL_BUSY);
 assert(CAN_Manager_SendMotorCurrent(&can1_manager,5,200)==HAL_OK);
 assert(CAN_Manager_SendMotorCurrent(&can1_manager,5,-300)==HAL_OK);
 assert(CAN_Manager_FlushTx(&can1_manager)==HAL_BUSY);
 assert(count==0 && can1_manager.tx_busy==2 && can1_manager.tx_err==0);
 submit_result=BSP_CAN_TX_ACCEPTED;
 assert(CAN_Manager_FlushTx(&can1_manager)==HAL_OK);
 expect(0,BSP_CAN_CHANNEL_1,0x2fe,0,-300);
 assert(CAN_Manager_FlushTx(&can1_manager)==HAL_OK && count==1);
 submit_result=BSP_CAN_TX_ERROR;
 assert(CAN_Manager_SendMotorCurrent(&can1_manager,5,400)==HAL_OK);
 assert(CAN_Manager_FlushTx(&can1_manager)==HAL_ERROR);
 assert(CAN_Manager_SendMotorCurrent(&can1_manager,5,500)==HAL_OK);
 submit_result=BSP_CAN_TX_ACCEPTED;
 assert(CAN_Manager_FlushTx(&can1_manager)==HAL_OK);
 expect(1,BSP_CAN_CHANNEL_1,0x2fe,0,500);
 /* Partial updates during BUSY preserve all latest slots of the shared frame. */
 submit_result=BSP_CAN_TX_BUSY;
 assert(CAN_Manager_SendMotorCurrent(&can1_manager,1,111)==HAL_OK);
 assert(CAN_Manager_SendMotorCurrent(&can1_manager,2,222)==HAL_OK);
 assert(CAN_Manager_FlushTx(&can1_manager)==HAL_BUSY);
 assert(CAN_Manager_SendMotorCurrent(&can1_manager,1,333)==HAL_OK);
 submit_result=BSP_CAN_TX_ACCEPTED;
 assert(CAN_Manager_FlushTx(&can1_manager)==HAL_OK);
 assert(frames[2].id==0x200 && frames[2].data[0]==1 && frames[2].data[1]==77);
 assert(frames[2].data[2]==0 && frames[2].data[3]==222);
 /* A fault discovered inside submission cannot retain a nonzero retry. */
 assert(CAN_Manager_SendMotorCurrent(&can1_manager,5,999)==HAL_OK);
 disarm_on_submit=true;
 assert(CAN_Manager_FlushTx(&can1_manager)==HAL_ERROR);
 disarm_on_submit=false; armed=true;
 assert(CAN_Manager_FlushTx(&can1_manager)==HAL_OK);
 expect(3,BSP_CAN_CHANNEL_1,0x2fe,0,0);
 count=0;
 /* CAN2 pitch slot coalesces to latest voltage command while hardware is busy. */
 assert(CAN_Manager_Init(&can2_manager,CAN_CHANNEL_2,&handles[1],robot,&registries[1])==HAL_OK);
 submit_result=BSP_CAN_TX_BUSY;
 assert(CAN_Manager_SendMotorCurrent(&can2_manager,8,100)==HAL_OK);
 assert(CAN_Manager_FlushTx(&can2_manager)==HAL_BUSY);
 assert(CAN_Manager_SendMotorCurrent(&can2_manager,8,-300)==HAL_OK);
 submit_result=BSP_CAN_TX_ACCEPTED;
 assert(CAN_Manager_FlushTx(&can2_manager)==HAL_OK);
 expect(0,BSP_CAN_CHANNEL_2,0x1ff,3,-300);
 assert(CAN_Manager_FlushTx(&can2_manager)==HAL_OK && count==1);
 count=0;
 puts("DJI latest pending: PASS (busy/error retry, overwrite, shared slots, fault purge)");
}

int main(void)
{
    const RobotConfig_t *robot = RobotConfig_Get();
    const MotorAdapterOps *adapter = DjiMotorAdapter_Get();
    for (uint8_t i = 0; i < robot->total_motor_count; ++i) {
        const MotorConfig_t *cfg = &robot->motor_configs[i];
        assert(adapter->validate(cfg) == ROBOT_STATUS_OK);
        contexts[cfg->motor_id] = (MotorContext_t){.initialized=true, .config=cfg, .type=cfg->type};
    }
    assert(CAN_Manager_Init(&can1_manager, CAN_CHANNEL_1, &handles[0], robot, &registries[0]) == HAL_OK);
    assert(CAN_Manager_Init(&can2_manager, CAN_CHANNEL_2, &handles[1], robot, &registries[1]) == HAL_OK);
    test_receive_overload(robot);
    test_latest_pending(robot);
    assert(adapter->command_current(5, 30000) == ROBOT_STATUS_OK);
    assert(adapter->command_current(8, -30000) == ROBOT_STATUS_OK);
    adapter->flush();
    assert(count == 2);
    expect(0, BSP_CAN_CHANNEL_1, 0x2FE, 0, DjiMotor_CommandLimit(contexts[5].config));
    expect(1, BSP_CAN_CHANNEL_2, 0x1FF, 3, -25000);
    adapter->flush(); assert(count == 2); // No unused command group emitted.
    assert(CAN_Manager_SendMotorCurrent(&can1_manager, 5, -32768) == HAL_OK);
    adapter->flush(); expect(2, BSP_CAN_CHANNEL_1, 0x2FE, 0, -DjiMotor_CommandLimit(contexts[5].config));
    assert(CAN_Manager_SendGM6020Current(&handles[0], 5, 32767) == HAL_OK);
    expect(3, BSP_CAN_CHANNEL_1, 0x2FE, 0, DjiMotor_CommandLimit(contexts[5].config));
    assert(adapter->stop(5) == ROBOT_STATUS_OK);
    adapter->flush(); expect(4, BSP_CAN_CHANNEL_1, 0x2FE, 0, 0);

    MotorConfig_t cfg = *contexts[5].config;
    cfg.protocol.dji.gm6020_mode = GM6020_COMMAND_VOLTAGE;
    cfg.can_tx_id = 0x2FE;
    assert(adapter->validate(&cfg) != ROBOT_STATUS_OK); // Reject mode/ID mismatch.
    cfg.can_tx_id = 0x2FF; cfg.tx_slot = 1;
    assert(adapter->validate(&cfg) != ROBOT_STATUS_OK);
    cfg.tx_slot = 0; cfg.protocol.dji.command_limit = 26000;
    assert(adapter->validate(&cfg) != ROBOT_STATUS_OK);
    cfg.protocol.dji.command_limit = 0;
    assert(DjiMotor_CommandLimit(&cfg) == 25000);
    cfg.protocol.dji.gm6020_mode = (GM6020CommandMode_e)99;
    assert(adapter->validate(&cfg) != ROBOT_STATUS_OK);

    cfg = *contexts[5].config;
    /* Keep optional current-mode coverage independent of the live robot mode. */
    cfg.protocol.dji.gm6020_mode = GM6020_COMMAND_CURRENT;
    cfg.protocol.dji.command_limit = 4096;
    cfg.can_tx_id = 0x2FE;
    RobotConfig_t one = {.motor_configs=&cfg, .total_motor_count=1};
    contexts[5].config = &cfg;
    assert(CAN_Manager_Init(&can1_manager, CAN_CHANNEL_1, &handles[0], &one, &registries[0]) == HAL_OK);
    assert(adapter->command_current(5, 30000) == ROBOT_STATUS_OK);
    assert(adapter->command_current(8, -30000) == ROBOT_STATUS_OK);
    adapter->flush();
    expect(5, BSP_CAN_CHANNEL_1, 0x2FE, 0, 4096);
    expect(6, BSP_CAN_CHANNEL_2, 0x1FF, 3, -25000);
    assert(CAN_Manager_SendGM6020Current(&handles[0], 5, -30000) == HAL_OK);
    expect(7, BSP_CAN_CHANNEL_1, 0x2FE, 0, -4096);

    cfg.can_rx_id = 0x205; cfg.can_tx_id = 0x1FE;
    cfg.protocol.dji.command_limit = 0;
    assert(CAN_Manager_Init(&can1_manager, CAN_CHANNEL_1, &handles[0], &one, &registries[0]) == HAL_OK);
    assert(CAN_Manager_SendMotorCurrent(&can1_manager, 5, -30000) == HAL_OK);
    adapter->flush(); expect(8, BSP_CAN_CHANNEL_1, 0x1FE, 0, -16384);
    cfg.can_tx_id = 0x1FF; // Bypass adapter and ensure CAN boundary also rejects mismatch.
    assert(CAN_Manager_SendMotorCurrent(&can1_manager, 5, 100) == HAL_ERROR);
    adapter->flush(); assert(count == 9);
    puts("DJI voltage/current command integration: PASS");
    return 0;
}

void BspCan_TraceConfigure(BspCanChannel c, uint16_t tx, uint16_t rx, uint8_t slot)
{ (void)c; (void)tx; (void)rx; (void)slot; }
void BspCan_TraceCommand(BspCanChannel c, uint16_t tx, uint8_t slot, int16_t raw)
{ (void)c; (void)tx; (void)slot; (void)raw; }
void BspCan_TraceRead(MotorTraceBatch *b) { memset(b, 0, sizeof(*b)); }

void BspCan_TraceFeedback(BspCanChannel c, uint16_t id, uint32_t ms, int32_t raw, uint32_t detail)
{ (void)c; (void)id; assert(ms == receive_tick && raw == -321 && (int32_t)detail == -123); }

BspCanTxResult BspCan_TryWrite(BspCanChannel c, uint16_t id, const uint8_t *d, uint8_t n)
{
 if (disarm_on_submit) { armed=false; return BSP_CAN_TX_ERROR; }
 if (submit_result != BSP_CAN_TX_ACCEPTED) return submit_result;
 return BspCan_Write(c,id,d,n) ? BSP_CAN_TX_ACCEPTED : BSP_CAN_TX_ERROR;
}
