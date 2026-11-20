/*
 * 声明单路 CAN 管理器及其统计数据。
 * 硬件句柄由冻结启动层传入，实际收发由 BSP 完成。
 */
#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "can.h"
#include "config_types.h"
#include "motor_registry.h"

// TX frame structure for aggregating motor commands
#define CAN_TX_FRAME_COUNT 5  // DJI current groups plus GM6020 voltage/current groups
typedef struct {
    uint16_t std_id;           // 0x200, 0x1FF, 0x2FF, 0x1FE, or 0x2FE
    int16_t currents[4];       // Currents for 4 motor slots
    uint32_t generated_ms[4];
    uint16_t ttl_ms[4], abort_ms[4];
    uint8_t valid_mask;         /* Slot ages survive mailbox acceptance; new output renews only its own slot. */
    uint8_t pending;           // latest per-slot values retained until mailbox acceptance
} CANTxFrame_t;

// CAN manager structure
typedef struct {
    CAN_HandleTypeDef *hcan;
    CAN_Channel_t channel;
    uint8_t initialized;

    // Motor registry for dynamic motor lookup
    MotorRegistry_t *registry;  // Pointer to avoid circular dependency

    // TX frame aggregation buffers
    CANTxFrame_t tx_frames[CAN_TX_FRAME_COUNT];

    // Debug counters
    uint32_t tx_ok;
    uint32_t tx_err;
    uint32_t tx_busy;           // deferred, latest software values remain pending
    uint32_t rx_frames;
    uint32_t last_rx_id;
    uint32_t last_tx_time;
    uint32_t last_rx_time;
} CAN_Manager_t;

/**
 * @brief Initialize CAN manager with robot configuration
 * @param manager CAN manager pointer
 * @param channel CAN channel (CAN_CHANNEL_1 or CAN_CHANNEL_2)
 * @param hcan CAN handle pointer
 * @param robot_config Robot configuration (for motor registry initialization)
 * @param registry_storage Pointer to registry storage (allocated by caller)
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_Init(CAN_Manager_t *manager,
                                  CAN_Channel_t channel,
                                  CAN_HandleTypeDef *hcan,
                                  const RobotConfig_t *robot_config,
                                  MotorRegistry_t *registry_storage);

/**
 * @brief Start CAN communication
 * @param manager CAN manager pointer
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_Start(CAN_Manager_t *manager);

/**
 * @brief Check if CAN is initialized
 * @param manager CAN manager pointer
 * @return true if initialized
 */
bool CAN_Manager_IsInitialized(const CAN_Manager_t *manager);

/**
 * @brief Get CAN handle
 * @param manager CAN manager pointer
 * @return CAN handle pointer
 */
CAN_HandleTypeDef* CAN_Manager_GetHandle(const CAN_Manager_t *manager);

/**
 * @brief Send 4 motor currents in one CAN frame to StdId 0x200/0x1FF
 * @param hcan CAN handle
 * @param std_id Standard ID (0x200 for motors 1-4 on CAN1, 0x1FF for 5-8 on CAN2)
 * @param i1 Current for slot 1 (motor 1 or 5)
 * @param i2 Current for slot 2 (motor 2 or 6)
 * @param i3 Current for slot 3 (motor 3 or 7)
 * @param i4 Current for slot 4 (motor 4 or 8)
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_SendMotorCurrents4(CAN_HandleTypeDef *hcan, uint16_t std_id,
                                                int16_t i1, int16_t i2, int16_t i3, int16_t i4);

/**
 * @brief Send a registered GM6020 hardware ID using its configured voltage/current mode
 * @param hcan CAN handle (typically hcan1)
 * @param motor_id GM6020 id in 1..7
 * @param current Native command counts, clamped to the per-motor protocol limit
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_SendGM6020Current(CAN_HandleTypeDef *hcan, uint8_t motor_id, int16_t current);

/**
 * @brief Send motor current by motor ID (new configurable API)
 *
 * Aggregates motor current into appropriate TX frame buffer based on
 * motor configuration. Call CAN_Manager_FlushTx() to actually send frames.
 *
 * @param manager CAN manager pointer
 * @param motor_id Motor ID
 * @param current Current value (will be clamped based on motor type)
 * @return HAL status
 */
HAL_StatusTypeDef CAN_Manager_SendMotorCurrent(CAN_Manager_t *manager,
                                              uint8_t motor_id,
                                              int16_t current);

/**
 * @brief Flush all pending TX frames
 *
 * Sends all TX frames that have been populated by CAN_Manager_SendMotorCurrent()
 * and clears the pending flags.
 *
 * @param manager CAN manager pointer
 * @return HAL status (returns error if any frame failed to send)
 */
HAL_StatusTypeDef CAN_Manager_FlushTx(CAN_Manager_t *manager);

/**
 * @brief Get CAN manager from handle (for reverse lookup)
 * @param hcan CAN handle
 * @return Pointer to CAN manager, or NULL if not found
 */
CAN_Manager_t* CAN_Manager_FromHandle(CAN_HandleTypeDef *hcan);

/**
 * @brief Process CAN receive callback
 * @param manager CAN manager pointer
 * @param hcan CAN handle that triggered the callback
 */
void CAN_Manager_ProcessCallback(CAN_Manager_t *manager, CAN_HandleTypeDef *hcan);

/**
 * @brief Global CAN callback function (to be called from HAL_CAN_RxFifo0MsgPendingCallback)
 * @param hcan CAN handle that triggered the callback
 */
void CAN_Manager_GlobalCallback(CAN_HandleTypeDef *hcan);

// Debug accessor helpers
uint32_t CAN_Manager_GetTxOk(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetTxErr(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetRxFrames(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetLastRxId(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetLastTxTime(const CAN_Manager_t *m);
uint32_t CAN_Manager_GetLastRxTime(const CAN_Manager_t *m);

#endif // CAN_MANAGER_H
