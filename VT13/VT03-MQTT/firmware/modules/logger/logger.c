/*
 * 输出带标签的文本和 CSV 日志，并按标签限流。
 * 日志失败不会改变控制输出，调试时要避免占满 USB 带宽。
 */
#include "logger.h"
#include "printing.h"
#include "bsp_time.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// ============================================================================
// Private Data Structures
// ============================================================================

// Rate limiter state for each tag
typedef struct {
    uint32_t last_log_time;     // Timestamp of last log (ms)
    uint32_t interval_ms;       // Minimum interval between logs (ms)
} LogRateLimiter_t;

// Rate limiter instances (one per tag)
static LogRateLimiter_t s_rate_limiters[LOG_TAG_COUNT];

// Tag name strings for CSV output
static const char* s_tag_names[LOG_TAG_COUNT] = {
    "SYS",      // LOG_TAG_SYS
    "CMD",      // LOG_TAG_CMD
    "CHA",      // LOG_TAG_CHA
    "GIM",      // LOG_TAG_GIM
    "SHO",      // LOG_TAG_SHO
    "SEN",      // LOG_TAG_SEN
    "MOT",      // LOG_TAG_MOT
    "IMU",      // LOG_TAG_IMU
    "CAN",      // LOG_TAG_CAN
    "VIS",      // LOG_TAG_VIS
    "RC",       // LOG_TAG_RC
    "DEBUG"     // LOG_TAG_DEBUG
};

// Log level strings for text output
static const char* s_level_strings[] = {
    "ERROR",    // LOG_LEVEL_ERROR
    "WARN",     // LOG_LEVEL_WARN
    "INFO",     // LOG_LEVEL_INFO
    "DEBUG"     // LOG_LEVEL_DEBUG
};

// ============================================================================
// Private Helper Functions
// ============================================================================

/**
 * Check if we should log based on rate limiting
 * Returns true if enough time has passed since last log
 */
static bool should_log(LogTag_t tag) {
    if (tag >= LOG_TAG_COUNT) {
        return false;  // Invalid tag
    }

    // No rate limit configured (interval = 0) => always log
    if (s_rate_limiters[tag].interval_ms == 0) {
        return true;
    }

    uint32_t now = BspTime_NowMs();
    uint32_t elapsed = now - s_rate_limiters[tag].last_log_time;

    if (elapsed >= s_rate_limiters[tag].interval_ms) {
        s_rate_limiters[tag].last_log_time = now;
        return true;
    }

    return false;  // Rate limited
}

// ============================================================================
// Public API Implementation
// ============================================================================

/**
 * Initialize the logger module
 */
void Logger_Init(void) {
    // Initialize all rate limiters with default interval
    for (int i = 0; i < LOG_TAG_COUNT; i++) {
        s_rate_limiters[i].last_log_time = 0;
        s_rate_limiters[i].interval_ms = LOG_RATE_DEFAULT;
    }
}

/**
 * Set the rate limit for a specific tag
 */
void Logger_SetRate(LogTag_t tag, uint32_t interval_ms) {
    if (tag < LOG_TAG_COUNT) {
        s_rate_limiters[tag].interval_ms = interval_ms;
    }
}

/**
 * Get the current rate limit for a tag
 */
uint32_t Logger_GetRate(LogTag_t tag) {
    if (tag < LOG_TAG_COUNT) {
        return s_rate_limiters[tag].interval_ms;
    }
    return 0;
}

/**
 * Log a text message with level prefix
 * Format: [TAG][LEVEL] message
 */
void Logger_Log(LogTag_t tag, LogLevel_t level, const char *fmt, ...) {
    if (tag >= LOG_TAG_COUNT || fmt == NULL) {
        return;
    }

    // Check rate limiting
    if (!should_log(tag)) {
        return;
    }

    char buf[LOG_BUFFER_SIZE];
    int offset = 0;

    // Add tag and level prefix for non-CSV levels
    if (level <= LOG_LEVEL_DEBUG) {
        offset = snprintf(buf, sizeof(buf), "[%s][%s] ",
                         s_tag_names[tag],
                         s_level_strings[level]);

        if (offset < 0 || offset >= (int)sizeof(buf)) {
            return;  // Buffer too small for prefix
        }
    }

    // Append user message
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + offset, sizeof(buf) - offset, fmt, ap);
    va_end(ap);

    if (n < 0) {
        return;  // Formatting error
    }

    // Add newline if not present
    int total_len = offset + n;
    if (total_len < (int)sizeof(buf) - 2) {
        if (buf[total_len - 1] != '\n') {
            buf[total_len] = '\r';
            buf[total_len + 1] = '\n';
            buf[total_len + 2] = '\0';
        }
    }

    // Output via existing printing infrastructure
    Debug_SendString(buf);
}

/**
 * Log CSV-formatted data with automatic timestamp
 * Format: TAG_NAME,timestamp_ms,user_fields...
 */
void Logger_CSV(LogTag_t tag, const char *fmt, ...) {
    if (tag >= LOG_TAG_COUNT || fmt == NULL) {
        return;
    }

    // Check rate limiting
    if (!should_log(tag)) {
        return;
    }

    char buf[LOG_BUFFER_SIZE];

    // Add CSV prefix: TAG_NAME,timestamp_ms,
    uint32_t timestamp = BspTime_NowMs();
    int offset = snprintf(buf, sizeof(buf), "%s,%lu,",
                         s_tag_names[tag],
                         (unsigned long)timestamp);

    if (offset < 0 || offset >= (int)sizeof(buf)) {
        return;  // Buffer too small for prefix
    }

    // Append user data fields
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + offset, sizeof(buf) - offset, fmt, ap);
    va_end(ap);

    if (n < 0) {
        return;  // Formatting error
    }

    // Add line terminator
    int total_len = offset + n;
    if (total_len < (int)sizeof(buf) - 2) {
        buf[total_len] = '\r';
        buf[total_len + 1] = '\n';
        buf[total_len + 2] = '\0';
    } else {
        // Buffer nearly full, just null-terminate
        buf[sizeof(buf) - 1] = '\0';
    }

    // Output via existing printing infrastructure
    Debug_SendString(buf);
}
