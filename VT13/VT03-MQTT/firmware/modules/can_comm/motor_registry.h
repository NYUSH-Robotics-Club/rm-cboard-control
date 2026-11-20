/*
 * 声明固定容量的电机配置查找表。
 * 查找只返回配置引用，不修改电机参数。
 */
#ifndef MOTOR_REGISTRY_H
#define MOTOR_REGISTRY_H

#include <stdint.h>
#include <stdbool.h>
#include "config_types.h"
#include "motor_feedback.h"

/**
 * @brief Motor Registry Module
 *
 * This module provides a runtime registry for motor configurations,
 * enabling dynamic lookup of motors by CAN RX ID or motor ID.
 * It bridges the gap between static compile-time configurations
 * and runtime CAN message processing.
 */

// Maximum number of motors supported
#define MOTOR_REGISTRY_MAX_MOTORS 16

/**
 * @brief Motor registry entry
 * Combines motor configuration with runtime feedback data
 */
typedef struct {
    const MotorConfig_t *config;     // Pointer to motor configuration
    bool active;                     // true if this slot is occupied

    // Feedback data storage (union to save memory)
    union {
        Motor_Feedback m3508_feedback;   // M3508/M2006 feedback
        struct {
            uint16_t angle_raw;          // Raw encoder value
            int16_t speed_rpm;           // Speed in RPM
            int16_t current;             // Motor current
            uint32_t last_update_time;   // Last update tick
        } gm6020_feedback;
    } feedback;
} MotorRegistryEntry_t;

/**
 * @brief Motor registry structure
 */
typedef struct {
    MotorRegistryEntry_t entries[MOTOR_REGISTRY_MAX_MOTORS];
    uint8_t motor_count;             // Number of registered motors
    CAN_Channel_t can_channel;       // CAN channel for this registry
} MotorRegistry_t;

/**
 * @brief Initialize motor registry from robot configuration
 *
 * Populates the registry with motors belonging to the specified CAN channel
 * from the robot configuration.
 *
 * @param registry Pointer to motor registry
 * @param robot_config Pointer to robot configuration
 * @param can_channel CAN channel to filter motors (CAN_CHANNEL_1 or CAN_CHANNEL_2)
 */
void MotorRegistry_Init(MotorRegistry_t *registry,
                       const RobotConfig_t *robot_config,
                       CAN_Channel_t can_channel);

/**
 * @brief Find motor configuration by CAN RX ID
 *
 * Performs O(n) linear search through the registry to find a motor
 * with matching CAN RX ID. Used during CAN receive processing.
 *
 * @param registry Pointer to motor registry
 * @param can_rx_id CAN RX ID to search for
 * @return Pointer to motor configuration, or NULL if not found
 */
const MotorConfig_t* MotorRegistry_FindByRxId(const MotorRegistry_t *registry,
                                             uint16_t can_rx_id);

/**
 * @brief Find motor configuration by motor ID
 *
 * Performs O(n) linear search through the registry to find a motor
 * with matching motor ID. Used when sending motor commands.
 *
 * @param registry Pointer to motor registry
 * @param motor_id Motor ID to search for
 * @return Pointer to motor configuration, or NULL if not found
 */
const MotorConfig_t* MotorRegistry_FindByMotorId(const MotorRegistry_t *registry,
                                                uint8_t motor_id);

/**
 * @brief Update motor feedback in registry
 *
 * Stores feedback data for a motor in the registry. Used by CAN manager
 * to cache latest feedback values.
 *
 * @param registry Pointer to motor registry
 * @param motor_id Motor ID
 * @param feedback Pointer to feedback data
 */
void MotorRegistry_UpdateFeedback(MotorRegistry_t *registry,
                                 uint8_t motor_id,
                                 const void *feedback);

/**
 * @brief Get motor feedback from registry
 *
 * Retrieves cached feedback data for a motor.
 *
 * @param registry Pointer to motor registry
 * @param motor_id Motor ID
 * @return Pointer to feedback data, or NULL if motor not found
 */
const void* MotorRegistry_GetFeedback(const MotorRegistry_t *registry,
                                     uint8_t motor_id);

/**
 * @brief Get number of registered motors
 *
 * @param registry Pointer to motor registry
 * @return Number of motors in registry
 */
uint8_t MotorRegistry_GetMotorCount(const MotorRegistry_t *registry);

#endif // MOTOR_REGISTRY_H
