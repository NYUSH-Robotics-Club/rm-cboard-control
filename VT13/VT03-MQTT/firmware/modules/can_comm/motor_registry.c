/*
 * 维护一路 CAN 上电机 ID 到配置的查找表。
 * 重复或超出容量的注册会失败，不会覆盖其他电机。
 */
#include "motor_registry.h"
#include <string.h>

/**
 * @brief Initialize motor registry from robot configuration
 */
void MotorRegistry_Init(MotorRegistry_t *registry,
                       const RobotConfig_t *robot_config,
                       CAN_Channel_t can_channel)
{
    if (registry == NULL || robot_config == NULL) {
        return;
    }

    // Clear registry
    memset(registry, 0, sizeof(MotorRegistry_t));
    registry->can_channel = can_channel;
    registry->motor_count = 0;

    // Populate registry with motors from this CAN channel
    for (uint8_t i = 0; i < robot_config->total_motor_count; i++) {
        const MotorConfig_t *motor_config = &robot_config->motor_configs[i];

        // Filter by CAN channel
        if (motor_config->can_channel == can_channel) {
            if (registry->motor_count >= MOTOR_REGISTRY_MAX_MOTORS) {
                // Registry full - this should not happen with proper configuration
                break;
            }

            // Add motor to registry
            MotorRegistryEntry_t *entry = &registry->entries[registry->motor_count];
            entry->config = motor_config;
            entry->active = true;
            registry->motor_count++;
        }
    }
}

/**
 * @brief Find motor configuration by CAN RX ID
 */
const MotorConfig_t* MotorRegistry_FindByRxId(const MotorRegistry_t *registry,
                                             uint16_t can_rx_id)
{
    if (registry == NULL) {
        return NULL;
    }

    for (uint8_t i = 0; i < registry->motor_count; i++) {
        const MotorRegistryEntry_t *entry = &registry->entries[i];
        if (entry->active && entry->config->can_rx_id == can_rx_id) {
            return entry->config;
        }
    }

    return NULL;  // Not found
}

/**
 * @brief Find motor configuration by motor ID
 */
const MotorConfig_t* MotorRegistry_FindByMotorId(const MotorRegistry_t *registry,
                                                uint8_t motor_id)
{
    if (registry == NULL) {
        return NULL;
    }

    for (uint8_t i = 0; i < registry->motor_count; i++) {
        const MotorRegistryEntry_t *entry = &registry->entries[i];
        if (entry->active && entry->config->motor_id == motor_id) {
            return entry->config;
        }
    }

    return NULL;  // Not found
}

/**
 * @brief Update motor feedback in registry
 */
void MotorRegistry_UpdateFeedback(MotorRegistry_t *registry,
                                 uint8_t motor_id,
                                 const void *feedback)
{
    if (registry == NULL || feedback == NULL) {
        return;
    }

    for (uint8_t i = 0; i < registry->motor_count; i++) {
        MotorRegistryEntry_t *entry = &registry->entries[i];
        if (entry->active && entry->config->motor_id == motor_id) {
            // Copy feedback data based on motor type
            if (entry->config->type == MOTOR_TYPE_M3508 || entry->config->type == MOTOR_TYPE_M2006) {
                memcpy(&entry->feedback.m3508_feedback, feedback, sizeof(Motor_Feedback));
            } else if (entry->config->type == MOTOR_TYPE_GM6020) {
                // GM6020 uses different feedback structure
                // For now, just copy raw data
                memcpy(&entry->feedback.gm6020_feedback, feedback, sizeof(entry->feedback.gm6020_feedback));
            }
            return;
        }
    }
}

/**
 * @brief Get motor feedback from registry
 */
const void* MotorRegistry_GetFeedback(const MotorRegistry_t *registry,
                                     uint8_t motor_id)
{
    if (registry == NULL) {
        return NULL;
    }

    for (uint8_t i = 0; i < registry->motor_count; i++) {
        const MotorRegistryEntry_t *entry = &registry->entries[i];
        if (entry->active && entry->config->motor_id == motor_id) {
            // Return pointer to appropriate feedback structure
            if (entry->config->type == MOTOR_TYPE_M3508 || entry->config->type == MOTOR_TYPE_M2006) {
                return &entry->feedback.m3508_feedback;
            } else if (entry->config->type == MOTOR_TYPE_GM6020) {
                return &entry->feedback.gm6020_feedback;
            }
        }
    }

    return NULL;  // Not found
}

/**
 * @brief Get number of registered motors
 */
uint8_t MotorRegistry_GetMotorCount(const MotorRegistry_t *registry)
{
    if (registry == NULL) {
        return 0;
    }
    return registry->motor_count;
}
