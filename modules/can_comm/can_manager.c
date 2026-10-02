/*
 * 配置一路 CAN、接收电机反馈并发布消息。
 * 本文件只解析已知 DJI 帧，未知厂商协议不在这里猜测。
 */
#include "can_manager.h"
#include "motor_registry.h"
#include "robot_config.h"
#include "message_center.h"
#include "can_comm.h"
#include <string.h>
#include "bsp_can.h"
#include "bsp_time.h"
#include "dji_motor_protocol.h"

static bool to_bsp_channel(CAN_Channel_t channel, BspCanChannel *bsp_channel)
{
    if (!bsp_channel) {
        return false;
    }
    if (channel == CAN_CHANNEL_1) {
        *bsp_channel = BSP_CAN_CHANNEL_1;
        return true;
    }
    if (channel == CAN_CHANNEL_2) {
        *bsp_channel = BSP_CAN_CHANNEL_2;
        return true;
    }
    return false;
}

HAL_StatusTypeDef CAN_Manager_Init(CAN_Manager_t *manager,
                                  CAN_Channel_t channel,
                                  CAN_HandleTypeDef *hcan,
                                  const RobotConfig_t *robot_config,
                                  MotorRegistry_t *registry_storage)
{
    if (manager == NULL || hcan == NULL || robot_config == NULL || registry_storage == NULL) {
        return HAL_ERROR;
    }

    // Clear manager structure
    memset(manager, 0, sizeof(CAN_Manager_t));
    manager->hcan = hcan;
    manager->channel = channel;

    BspCanChannel bsp_channel;
    if (!to_bsp_channel(channel, &bsp_channel) ||
        !BspCan_MatchesNativeHandle(bsp_channel, hcan)) {
        return HAL_ERROR;
    }

    // Initialize motor registry
    manager->registry = registry_storage;
    MotorRegistry_Init(manager->registry, robot_config, channel);

    // Initialize TX frame buffers
    manager->tx_frames[0].std_id = 0x200;
    manager->tx_frames[1].std_id = 0x1FF;
    manager->tx_frames[2].std_id = 0x2FF;
    manager->tx_frames[3].std_id = 0x1FE;
    manager->tx_frames[4].std_id = 0x2FE;

    for (unsigned i = 0U; i < robot_config->total_motor_count; ++i) {
        const MotorConfig_t *motor = &robot_config->motor_configs[i];
        if (motor->role == MOTOR_ROLE_GIMBAL_YAW && motor->vendor == MOTOR_VENDOR_DJI &&
            motor->type == MOTOR_TYPE_GM6020 && motor->can_channel == channel)
            BspCan_TraceConfigure(bsp_channel, motor->can_tx_id, motor->can_rx_id, motor->tx_slot);
    }
    manager->initialized = 1;
    return HAL_OK;
}

HAL_StatusTypeDef CAN_Manager_Start(CAN_Manager_t *manager)
{
    if (manager == NULL || !manager->initialized) return HAL_ERROR;
    BspCanChannel channel;
    return to_bsp_channel(manager->channel, &channel) && BspCan_Start(channel)
               ? HAL_OK
               : HAL_ERROR;
}

bool CAN_Manager_IsInitialized(const CAN_Manager_t *manager)
{
    if (manager == NULL) return false;
    return manager->initialized;
}

CAN_HandleTypeDef* CAN_Manager_GetHandle(const CAN_Manager_t *manager)
{
    if (manager == NULL || !manager->initialized) return NULL;
    return manager->hcan;
}

void CAN_Manager_ProcessCallback(CAN_Manager_t *manager, CAN_HandleTypeDef *hcan)
{
    if (manager == NULL || !manager->initialized || hcan != manager->hcan) return;
    BspCanChannel channel;
    BspCanFrame rx;
    if (!to_bsp_channel(manager->channel, &channel) || !BspCan_Read(channel, &rx)) return;
    uint32_t current_tick = BspTime_NowMs();
    manager->rx_frames++;
    manager->last_rx_id = rx.standard_id;
    manager->last_rx_time = current_tick;

    /* 已注册 DJI 只发布解码后的最新反馈，避免每帧重复占用两条消息。
     * 非 DJI、未知 ID 及非标准帧仍走原始帧路径，保留其他适配器的协议入口。
     */
    const MotorConfig_t *motor = NULL;
    if (rx.is_standard_frame && rx.is_data_frame && rx.length == 8U) {
        motor = MotorRegistry_FindByRxId(manager->registry, rx.standard_id);
    }
    if (!motor || motor->vendor != MOTOR_VENDOR_DJI) {
        CanRxFrame f = {
            .channel = manager->channel,
            .std_id = (uint16_t)rx.standard_id,
            .dlc = rx.length,
            .is_standard_frame = rx.is_standard_frame,
            .is_data_frame = rx.is_data_frame,
            .tick_ms = current_tick
        };
        memcpy(f.data, rx.data, rx.length);
        (void)MsgCenter_Publish(TOPIC_CAN_RX, &f, sizeof(f));
        return;
    }

    // Parse feedback based on motor type
    if (motor->type == MOTOR_TYPE_M3508 || motor->type == MOTOR_TYPE_M2006) {
        // M3508/M2006 feedback format:
        // Bytes [0-1]: Encoder angle (0-8191)
        // Bytes [2-3]: Speed (RPM, signed)
        // Bytes [4-5]: Current (signed)
        // Byte  [6]:   Temperature (degrees C)
        uint16_t angle   = (uint16_t)((rx.data[0] << 8) | rx.data[1]);
        int16_t  speed   = (int16_t)((rx.data[2] << 8) | rx.data[3]);
        int16_t  current = (int16_t)((rx.data[4] << 8) | rx.data[5]);
        uint8_t  temp    = rx.data[6];

        // Publish to message center
        MotorFeedbackEvent ev = {
            .id = motor->motor_id,
            .angle = angle,
            .speed = speed,
            .current = current,
            .temp = temp,
            .tick_ms = current_tick
        };
        (void)MsgCenter_PublishLatest(TOPIC_MOTOR_FEEDBACK, motor->motor_id,
                                     MC_LATEST_STATE, &ev, sizeof(ev));
    }
    else if (motor->type == MOTOR_TYPE_GM6020) {
        // GM6020 feedback format:
        // Bytes [0-1]: Encoder angle (0-8191)
        // Bytes [2-3]: Speed (RPM, signed)
        // Bytes [4-5]: Current (signed)
        // Byte  [6]:   Temperature (degrees C)
        uint16_t angle_raw = (uint16_t)((rx.data[0] << 8) | rx.data[1]);
        int16_t  speed_rpm = (int16_t)((rx.data[2] << 8) | rx.data[3]);
        int16_t  current   = (int16_t)((rx.data[4] << 8) | rx.data[5]);

        /* Record every decoded receive before latest-state coalescing. */
        BspCan_TraceFeedback(channel, (uint16_t)rx.standard_id, current_tick,
                             current, (uint32_t)(int32_t)speed_rpm);

        // Publish to message center
        GM6020FeedbackEvent gev = {
            .id = motor->motor_id,
            .angle = angle_raw,
            .speed = speed_rpm,
            .tick_ms = current_tick,
            .current = current
        };
        (void)MsgCenter_PublishLatest(TOPIC_GM6020_FEEDBACK, motor->motor_id,
                                     MC_LATEST_STATE, &gev, sizeof(gev));
    }
    // ========== END: Dynamic feedback processing ==========
}

extern CAN_Manager_t can1_manager;
extern CAN_Manager_t can2_manager;

void CAN_Manager_GlobalCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == CAN_Manager_GetHandle(&can1_manager)) {
        CAN_Manager_ProcessCallback(&can1_manager, hcan);
    } else if (hcan == CAN_Manager_GetHandle(&can2_manager)) {
        CAN_Manager_ProcessCallback(&can2_manager, hcan);
    }
}

HAL_StatusTypeDef CAN_Manager_SendMotorCurrents4(CAN_HandleTypeDef *hcan, uint16_t std_id,
                                                int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
    if (hcan == NULL) return HAL_ERROR;
    static uint32_t last_tx_tick = 0;   
    uint32_t now = BspTime_NowMs();
    if (now - last_tx_tick < 4) {
        return HAL_OK;
    }
    uint8_t d[8];
    d[0] = (uint8_t)(i1 >> 8); d[1] = (uint8_t)i1;
    d[2] = (uint8_t)(i2 >> 8); d[3] = (uint8_t)i2;
    d[4] = (uint8_t)(i3 >> 8); d[5] = (uint8_t)i3;
    d[6] = (uint8_t)(i4 >> 8); d[7] = (uint8_t)i4;
    CAN_Manager_t *m = CAN_Manager_FromHandle(hcan);
    BspCanChannel channel;
    HAL_StatusTypeDef st = m && to_bsp_channel(m->channel, &channel) &&
                           BspCan_Write(channel, std_id, d, sizeof(d))
                               ? HAL_OK
                               : HAL_ERROR;
    if (m) {
        if (st == HAL_OK) m->tx_ok++; else m->tx_err++;
        m->last_tx_time = BspTime_NowMs();
    }
    return st;
}

HAL_StatusTypeDef CAN_Manager_SendGM6020Current(CAN_HandleTypeDef *hcan, uint8_t motor_id, int16_t current)
{
    CAN_Manager_t *m = CAN_Manager_FromHandle(hcan);
    if (!m || !m->initialized || motor_id < 1U || motor_id > 7U) return HAL_ERROR;
    const MotorConfig_t *motor = MotorRegistry_FindByRxId(m->registry, 0x204U + motor_id);
    if (!motor || motor->type != MOTOR_TYPE_GM6020) return HAL_ERROR;
    /* Legacy hardware-ID callers must obey the same mode and limit as logical IDs. */
    if (CAN_Manager_SendMotorCurrent(m, motor->motor_id, current) != HAL_OK) return HAL_ERROR;
    return CAN_Manager_FlushTx(m);
}

/**
 * @brief Send motor current by motor ID (new configurable API)
 */
HAL_StatusTypeDef CAN_Manager_SendMotorCurrent(CAN_Manager_t *manager,
                                              uint8_t motor_id,
                                              int16_t current)
{
    if (manager == NULL || !manager->initialized || manager->registry == NULL) {
        return HAL_ERROR;
    }

    // Find motor configuration
    const MotorConfig_t *motor = MotorRegistry_FindByMotorId(manager->registry, motor_id);
    if (motor == NULL) {
        return HAL_ERROR;  // Motor not found in this CAN channel
    }

    // Enforce the configured protocol even when callers bypass MotorService.
    int16_t limit = DjiMotor_CommandLimit(motor);
    if (limit == 0) return HAL_ERROR;
    if (current > limit) current = limit;
    if (current < -limit) current = (int16_t)-limit;

    // Find appropriate TX frame
    CANTxFrame_t *tx_frame = NULL;
    for (uint8_t i = 0; i < CAN_TX_FRAME_COUNT; i++) {
        if (manager->tx_frames[i].std_id == motor->can_tx_id) {
            tx_frame = &manager->tx_frames[i];
            break;
        }
    }

    if (tx_frame == NULL) {
        return HAL_ERROR;  // Unsupported TX ID
    }

    // Aggregate current into appropriate slot
    if (motor->tx_slot < 4) {
        BspCanChannel channel;
        if (to_bsp_channel(manager->channel, &channel))
            BspCan_TraceCommand(channel, motor->can_tx_id, motor->tx_slot, current);
        tx_frame->currents[motor->tx_slot] = current;
        tx_frame->pending = 1;  // Mark frame as pending
    } else {
        return HAL_ERROR;  // Invalid slot
    }

    return HAL_OK;
}

/**
 * @brief Flush all pending TX frames
 */
HAL_StatusTypeDef CAN_Manager_FlushTx(CAN_Manager_t *manager)
{
    if (manager == NULL || !manager->initialized) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef result = HAL_OK;

    /* Recovery must not retain a pre-fault nonzero command in the software queue. */
    if (!BspCan_OutputsArmed()) {
        for (uint8_t i = 0U; i < CAN_TX_FRAME_COUNT; ++i) {
            memset(manager->tx_frames[i].currents, 0, sizeof(manager->tx_frames[i].currents));
        }
    }

    // Send all pending frames
    for (uint8_t i = 0; i < CAN_TX_FRAME_COUNT; i++) {
        CANTxFrame_t *tx_frame = &manager->tx_frames[i];

        if (tx_frame->pending) {
            uint8_t data[8] = {0};

            // Pack currents into data buffer (big-endian)
            for (uint8_t slot = 0; slot < 4; slot++) {
                data[slot * 2 + 0] = (uint8_t)((tx_frame->currents[slot] >> 8) & 0xFF);
                data[slot * 2 + 1] = (uint8_t)(tx_frame->currents[slot] & 0xFF);
            }

            BspCanChannel channel;
            HAL_StatusTypeDef status =
                to_bsp_channel(manager->channel, &channel) &&
                        BspCan_Write(channel, tx_frame->std_id, data, sizeof(data))
                    ? HAL_OK
                    : HAL_ERROR;
            if (status == HAL_OK) {
                manager->tx_ok++;
            } else {
                manager->tx_err++;
                result = HAL_ERROR;  // Mark as error but continue sending other frames
            }
            manager->last_tx_time = BspTime_NowMs();

            // Clear frame for next cycle
            memset(tx_frame->currents, 0, sizeof(tx_frame->currents));
            tx_frame->pending = 0;
        }
    }

    return result;
}

/**
 * @brief Get CAN manager from handle (for reverse lookup)
 */
CAN_Manager_t* CAN_Manager_FromHandle(CAN_HandleTypeDef *hcan)
{
    extern CAN_Manager_t can1_manager;
    extern CAN_Manager_t can2_manager;

    if (hcan == can1_manager.hcan) {
        return &can1_manager;
    } else if (hcan == can2_manager.hcan) {
        return &can2_manager;
    }
    return NULL;
}

uint32_t CAN_Manager_GetTxOk(const CAN_Manager_t *m){ return m?m->tx_ok:0; }
uint32_t CAN_Manager_GetTxErr(const CAN_Manager_t *m){ return m?m->tx_err:0; }
uint32_t CAN_Manager_GetRxFrames(const CAN_Manager_t *m){ return m?m->rx_frames:0; }
uint32_t CAN_Manager_GetLastRxId(const CAN_Manager_t *m){ return m?m->last_rx_id:0; }
uint32_t CAN_Manager_GetLastTxTime(const CAN_Manager_t *m){ return m?m->last_tx_time:0; }
uint32_t CAN_Manager_GetLastRxTime(const CAN_Manager_t *m){ return m?m->last_rx_time:0; }
