/*
 * 声明统一日志标签、级别和便捷宏。
 * 被关闭的标签在编译后不执行输出。
 */
#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "logger_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Log Tags (Subsystems)
// ============================================================================

typedef enum {
    LOG_TAG_SYS = 0,    // System/boot messages
    LOG_TAG_CMD,        // Command controller
    LOG_TAG_CHA,        // Chassis controller
    LOG_TAG_GIM,        // Gimbal controller
    LOG_TAG_SHO,        // Shooter controller
    LOG_TAG_SEN,        // Sentry controller (swerve chassis variant)
    LOG_TAG_MOT,        // Motor driver
    LOG_TAG_IMU,        // IMU sensors
    LOG_TAG_CAN,        // CAN communication
    LOG_TAG_VIS,        // Vision communication
    LOG_TAG_RC,         // Remote control
    LOG_TAG_DEBUG,      // General debug
    LOG_TAG_COUNT       // Total number of tags
} LogTag_t;

// ============================================================================
// Log Levels
// ============================================================================

typedef enum {
    LOG_LEVEL_ERROR = 0,  // Critical errors
    LOG_LEVEL_WARN,       // Warnings
    LOG_LEVEL_INFO,       // Informational messages
    LOG_LEVEL_DEBUG,      // Verbose debug output
    LOG_LEVEL_CSV         // CSV data for plotting/analysis
} LogLevel_t;

// ============================================================================
// Public API
// ============================================================================

/**
 * Initialize the logger module
 * Must be called once during system initialization
 */
void Logger_Init(void);

/**
 * Set the rate limit for a specific tag (in milliseconds)
 * @param tag The log tag to configure
 * @param interval_ms Minimum interval between log messages (0 = no limit)
 */
void Logger_SetRate(LogTag_t tag, uint32_t interval_ms);

/**
 * Get the current rate limit for a tag
 * @param tag The log tag to query
 * @return Current interval in milliseconds
 */
uint32_t Logger_GetRate(LogTag_t tag);

/**
 * Log a text message with automatic rate limiting
 * Format: [TAG][LEVEL] message
 *
 * @param tag The log tag/subsystem
 * @param level The log level (ERROR, WARN, INFO, DEBUG)
 * @param fmt Printf-style format string
 * @param ... Variable arguments for format string
 */
void Logger_Log(LogTag_t tag, LogLevel_t level, const char *fmt, ...);

/**
 * Log CSV-formatted data with automatic rate limiting
 * Format: TAG_NAME,timestamp_ms,user_fields...
 *
 * @param tag The log tag/subsystem
 * @param fmt Printf-style format string for data fields (timestamp auto-added)
 * @param ... Variable arguments for format string
 */
void Logger_CSV(LogTag_t tag, const char *fmt, ...);

// ============================================================================
// Convenience Macros
// ============================================================================

// Compile-time tag enable/disable check
#define LOG_TAG_ENABLED(tag) \
    ((tag == LOG_TAG_SYS && LOG_ENABLE_SYS) || \
     (tag == LOG_TAG_CMD && LOG_ENABLE_CMD) || \
     (tag == LOG_TAG_CHA && LOG_ENABLE_CHA) || \
     (tag == LOG_TAG_GIM && LOG_ENABLE_GIM) || \
     (tag == LOG_TAG_SHO && LOG_ENABLE_SHO) || \
     (tag == LOG_TAG_SEN && LOG_ENABLE_SEN) || \
     (tag == LOG_TAG_MOT && LOG_ENABLE_MOT) || \
     (tag == LOG_TAG_IMU && LOG_ENABLE_IMU) || \
     (tag == LOG_TAG_CAN && LOG_ENABLE_CAN) || \
     (tag == LOG_TAG_VIS && LOG_ENABLE_VIS) || \
     (tag == LOG_TAG_RC  && LOG_ENABLE_RC) || \
     (tag == LOG_TAG_DEBUG && LOG_ENABLE_DEBUG))

// Text logging macros with compile-time optimization
#define LOG_ERROR(tag, fmt, ...) \
    do { if (LOG_TAG_ENABLED(tag)) Logger_Log(tag, LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__); } while(0)

#define LOG_WARN(tag, fmt, ...) \
    do { if (LOG_TAG_ENABLED(tag)) Logger_Log(tag, LOG_LEVEL_WARN, fmt, ##__VA_ARGS__); } while(0)

#define LOG_INFO(tag, fmt, ...) \
    do { if (LOG_TAG_ENABLED(tag)) Logger_Log(tag, LOG_LEVEL_INFO, fmt, ##__VA_ARGS__); } while(0)

#define LOG_DEBUG(tag, fmt, ...) \
    do { if (LOG_TAG_ENABLED(tag)) Logger_Log(tag, LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__); } while(0)

// CSV logging macro with compile-time optimization
#define LOG_CSV(tag, fmt, ...) \
    do { if (LOG_TAG_ENABLED(tag)) Logger_CSV(tag, fmt, ##__VA_ARGS__); } while(0)

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H
