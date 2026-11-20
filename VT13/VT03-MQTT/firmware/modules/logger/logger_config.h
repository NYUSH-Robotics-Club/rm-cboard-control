/*
 * 保存日志标签开关、默认限流和缓冲区大小。
 * 修改开关后需要重新编译固件。
 */
#ifndef LOGGER_CONFIG_H
#define LOGGER_CONFIG_H

/*
 * Logger Module Compile-time Configuration
 *
 * Enable/disable log tags at compile time to reduce flash/RAM usage.
 * Disabled tags compile to no-op (zero overhead).
 */

// ============================================================================
// Log Tag Enable/Disable (1 = enabled, 0 = disabled)
// ============================================================================

#define LOG_ENABLE_SYS    0  // System/boot messages
#define LOG_ENABLE_CMD    0  // Command controller
#define LOG_ENABLE_CHA    0  // Chassis controller
#define LOG_ENABLE_GIM    1  // Gimbal controller - ENABLED for YAW plotting
#define LOG_ENABLE_SHO    0  // Shooter controller
#define LOG_ENABLE_SEN    0  // Sentry controller
#define LOG_ENABLE_MOT    0  // Motor driver
#define LOG_ENABLE_IMU    0  // IMU sensors
#define LOG_ENABLE_CAN    0  // CAN communication
#define LOG_ENABLE_VIS    0  // Vision communication
#define LOG_ENABLE_RC     0  // Remote control
#define LOG_ENABLE_DEBUG  0  // General debug

// ============================================================================
// Default Rate Limits (milliseconds)
// ============================================================================

// Standard rate for most CSV data (10Hz)
#define LOG_RATE_DEFAULT   100

// Fast rate for high-frequency data like gimbal PID tuning (20Hz)
#define LOG_RATE_FAST      50

// Slow rate for infrequent status messages (2Hz)
#define LOG_RATE_SLOW      500

// ============================================================================
// Buffer Configuration
// ============================================================================

// Maximum log message size (doubled from original 128 bytes)
#define LOG_BUFFER_SIZE    256

#endif // LOGGER_CONFIG_H
