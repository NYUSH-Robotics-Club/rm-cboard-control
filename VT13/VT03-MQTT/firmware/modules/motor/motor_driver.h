/*
 * 声明现有 DJI 电机驱动的兼容接口和状态。
 * 角度为编码器刻度、速度为 RPM、电流为驱动器命令单位。
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "config_types.h"
#include "pid.h"

/**
 * @brief Generic Motor Driver Module
 *
 * This module provides a unified driver interface for all motor types
 * (M3508, GM6020, M2006), replacing the old gm6020_motor.c.
 * Motor contexts are initialized from configuration instead of hardcoded values.
 */

// Maximum number of motors supported
#define MOTOR_DRIVER_MAX_MOTORS 16

/* Native command counts, selected per motor; zero means invalid configuration. */
int16_t MotorDriver_GetCommandLimit(uint8_t motor_id);

/**
 * @brief Motor context structure
 * Unified structure for all motor types, contains feedback data,
 * control state, and PID controllers.
 */
typedef struct {
    // Motor identification
    uint8_t motor_id;               // Logical motor ID (0-15)
    MotorType_e type;               // Motor type
    MotorRole_e role;               // Motor role
    bool initialized;               // Initialization flag

    // Configuration reference
    const MotorConfig_t *config;    // Pointer to motor configuration

    // Feedback data (updated from CAN)
    uint16_t angle_raw;             // Raw encoder value (0-8191)
    int16_t speed_rpm;              // Speed in RPM
    int16_t feedback_current;       // Actual motor current
    uint32_t last_feedback_time;    // Last feedback timestamp

    // Control state
    float angle_target;             // Target angle (encoder units)
    bool angle_initialized;         // true if initial angle has been set

    // Computed values
    float target_angle_rad;         // Current angle in radians
    float angle_correction;         // Angle correction for gimbal
    float angle_correction_ramp;    // Ramped angle correction

    // PID controllers
    PID_Controller pid_outer;       // Outer loop (angle/position)
    PID_Controller pid_inner;       // Inner loop (speed)

    // Motor-specific state (used by application layer)
    union {
        // GM6020-specific (gimbal motors)
        struct {
            float w_chassis_raw;        // Raw chassis angular velocity
            float joystick_sensitivity; // Joystick sensitivity
        } gm6020;

        // M3508-specific (chassis/shooter motors)
        struct {
            float ramped_target_speed;  // Ramped target speed
        } m3508;
    } state;

} MotorContext_t;

/**
 * @brief Initialize motor driver module from robot configuration
 *
 * This function should be called once during system initialization.
 * It loads the robot configuration and sets up all motors.
 * Application layer does not need to access configuration directly.
 */
void MotorDriver_ModuleInit(void);

/**
 * @brief Initialize a motor from configuration
 *
 * Initializes a motor context from the provided configuration.
 * Sets up PID controllers, angle limits, and initial state.
 *
 * @param motor_id Motor ID (must match config->motor_id)
 * @param config Pointer to motor configuration
 * @return true if successful, false if motor_id invalid
 */
bool MotorDriver_Init(uint8_t motor_id, const MotorConfig_t *config);

/**
 * @brief Update motor feedback from CAN message
 *
 * Called by CAN manager or message center callback to update motor state.
 * For GM6020: angle, speed, current
 * For M3508: angle, speed, current, temperature
 *
 * @param motor_id Motor ID
 * @param angle_raw Raw encoder value (0-8191)
 * @param speed_rpm Speed in RPM
 * @param current Motor current
 * @param timestamp Feedback timestamp
 */
void MotorDriver_UpdateFeedback(uint8_t motor_id,
                                uint16_t angle_raw,
                                int16_t speed_rpm,
                                int16_t current,
                                uint32_t timestamp);

/**
 * @brief Compute motor current command
 *
 * Generic PID control computation for all motor types.
 * Supports:
 * - Angle control (outer loop)
 * - Speed control (inner loop)
 * - Gravity compensation (for pitch motors)
 * - Current limiting based on motor type
 *
 * @param motor_id Motor ID
 * @param target_angle Target angle (encoder units) or target speed (RPM)
 * @param use_angle_control true for angle control, false for speed control
 * @return Computed current command (clamped to motor limits)
 */
int16_t MotorDriver_ComputeCurrent(uint8_t motor_id,
                                   float target_angle,
                                   bool use_angle_control);

/**
 * @brief Get motor context
 *
 * Returns pointer to motor context for application layer access.
 * Application can read motor state but should not modify it directly.
 *
 * @param motor_id Motor ID
 * @return Pointer to motor context, or NULL if motor_id invalid
 */
MotorContext_t* MotorDriver_GetContext(uint8_t motor_id);

/**
 * @brief Check if motor is initialized
 *
 * @param motor_id Motor ID
 * @return true if motor is initialized and has received feedback
 */
bool MotorDriver_IsInitialized(uint8_t motor_id);

/**
 * @brief Set motor angle target (for gimbal motors)
 *
 * Convenience function for gimbal control.
 *
 * @param motor_id Motor ID
 * @param target_angle Target angle (encoder units)
 */
void MotorDriver_SetAngleTarget(uint8_t motor_id, float target_angle);

/**
 * @brief Reset motor PID integrals
 *
 * Useful when motor is stopped or control mode changes.
 *
 * @param motor_id Motor ID
 */
void MotorDriver_ResetPID(uint8_t motor_id);

/**
 * @brief Find motors by role
 *
 * Application layer uses this to discover which motors are available
 * for a specific function without knowing configuration details.
 *
 * @param role Motor role to search for
 * @param motor_ids Output array to store found motor IDs
 * @param max_count Maximum number of motors to find (size of motor_ids array)
 * @return Number of motors found
 */
uint8_t MotorDriver_FindByRole(MotorRole_e role, uint8_t *motor_ids, uint8_t max_count);

/**
 * @brief Send current command to a motor
 *
 * Application layer calls this to command a motor. The module layer
 * handles CAN transmission internally. Application doesn't need to know
 * about CAN managers or channels.
 *
 * @param motor_id Motor ID
 * @param current Current command value
 */
void MotorDriver_SendCurrent(uint8_t motor_id, int16_t current);

/**
 * @brief Flush all pending motor current commands
 *
 * Should be called once per control cycle after all SendCurrent calls.
 * This triggers actual CAN transmission for all buffered commands.
 */
void MotorDriver_FlushAll(void);

#endif // MOTOR_DRIVER_H
