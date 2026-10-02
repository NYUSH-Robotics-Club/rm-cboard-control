/*
 * 管理两路板级 CAN 收发与诊断。
 * 上层只看到标准帧和通道号，不直接访问 HAL 寄存器。
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>
#include "motor_trace.h"

typedef enum {
    BSP_CAN_CHANNEL_1 = 1,
    BSP_CAN_CHANNEL_2 = 2
} BspCanChannel;

typedef struct {
    uint32_t error;
    uint32_t error_status_register;
    uint32_t transmit_status_register;
    uint32_t receive_fifo0_register;
    uint32_t free_tx_mailboxes;
} BspCanDiagnostics;

typedef struct {
    uint32_t standard_id;
    uint8_t length;
    uint8_t data[8];
    bool is_standard_frame;
    bool is_data_frame;
} BspCanFrame;

/* Configure before CAN starts. One configured DJI yaw is traced; no control changes. */
void BspCan_TraceConfigure(BspCanChannel channel, uint16_t tx_id, uint16_t rx_id, uint8_t slot);
void BspCan_TraceCommand(BspCanChannel channel, uint16_t tx_id, uint8_t slot, int16_t raw);
/* Opaque values decoded by the protocol layer, timestamped at reception. */
void BspCan_TraceFeedback(BspCanChannel channel, uint16_t rx_id, uint32_t ms, int32_t raw, uint32_t detail);
void BspCan_TraceRead(MotorTraceBatch *batch);

/* 使用当前板卡的固定过滤器配置启动通道。 */
bool BspCan_Start(BspCanChannel channel);

/* 从通道的 FIFO0 读取一帧；无数据或硬件错误均返回 false。 */
bool BspCan_Read(BspCanChannel channel, BspCanFrame *frame);

/* 发送 0～8 字节的标准数据帧。 */
bool BspCan_Write(BspCanChannel channel,
                  uint16_t standard_id,
                  const uint8_t *data,
                  uint8_t length);

/* 仅为冻结中断回调的旧句柄提供通道识别。 */
bool BspCan_MatchesNativeHandle(BspCanChannel channel, const void *native_handle);

bool BspCan_ReadDiagnostics(BspCanChannel channel, BspCanDiagnostics *diagnostics);

/* Recovery runs from the main loop, never from the receive interrupt. */
typedef struct {
    uint32_t phase; /* 0=ready, 1=abort, 2=init, 3=leave init, 4=bus sync, 5=retry wait */
    uint32_t phase_since_ms;
    uint32_t fault_count;
    uint32_t recovery_count;
    uint32_t timeout_count;
    uint32_t first_fault_esr;
    uint32_t last_error_esr;
} BspCanRecovery;

void BspCan_Service(uint32_t now_ms);
const BspCanRecovery *BspCan_GetRecovery(BspCanChannel channel);
/* Both buses must be healthy for 500 ms. Arming additionally requires operator neutral. */
bool BspCan_RecoveryReady(uint32_t now_ms);
bool BspCan_TryArm(uint32_t now_ms);
bool BspCan_OutputsArmed(void);

#endif
