/**
 * @file vision_config.h
 * @brief Vision communication configuration and parameter adjustment
 * @note The configuration API exists, but only the enhanced receive path reads
 *       most fields. The active USB callback does not use that path yet.
 */

#ifndef VISION_CONFIG_H
#define VISION_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// 1. Data Validation Parameters (数据验证参数)
// =============================================================================

// Angle valid range (radians) - 角度有效范围
#define VISION_YAW_MIN      (-3.14159f)     // -180 degrees
#define VISION_YAW_MAX      (3.14159f)      //  180 degrees
#define VISION_PITCH_MIN    (-0.785398f)    // -45 degrees (typical gimbal limit)
#define VISION_PITCH_MAX    (0.785398f)     //  45 degrees

// Data timeout (milliseconds) - 数据超时时间
#define VISION_DATA_TIMEOUT_MS  200         // Consider data stale after 200ms

// =============================================================================
// 2. Communication Parameters (通信参数)
// =============================================================================

// Send frequency control - 发送频率控制
#define VISION_SEND_INTERVAL_MS 10          // 100Hz (adjustable: 5-50ms)

// Retransmission parameters - 重传参数
#define VISION_MAX_RETRIES      3           // Max retransmission attempts
#define VISION_RETRY_INTERVAL_MS 10         // Interval between retries

// =============================================================================
// 3. Filter Parameters (滤波参数)
// =============================================================================

// Moving average filter window size - 滑动平均滤波窗口大小
#define VISION_FILTER_WINDOW    5           // Number of samples (1-10)
// Higher value = smoother but more delay
// Lower value = faster response but more noise

// Exponential Moving Average (EMA) alpha - 指数移动平均系数
#define VISION_EMA_ALPHA        0.3f        // Range: 0.0-1.0
// Higher value = more weight on new data (faster response)
// Lower value = more weight on history (smoother)

// =============================================================================
// 4. Data Fusion Parameters (数据融合参数)
// =============================================================================

// Weight for vision data vs IMU data - 视觉数据与IMU数据的融合权重
#define VISION_FUSION_WEIGHT    0.7f        // Range: 0.0-1.0
// 1.0 = trust vision completely
// 0.0 = ignore vision (IMU only)

// Threshold for using vision data (convergence) - 收敛阈值
#define VISION_CONVERGE_THRESHOLD 0.05f     // radians (~3 degrees)

// =============================================================================
// Configuration Structure (Runtime adjustable parameters)
// =============================================================================

typedef struct {
    // Data validation
    float yaw_min;
    float yaw_max;
    float pitch_min;
    float pitch_max;
    uint32_t data_timeout_ms;

    // Communication
    uint32_t send_interval_ms;
    uint8_t max_retries;
    uint32_t retry_interval_ms;

    // Filtering
    uint8_t filter_window;
    float ema_alpha;

    // Data fusion
    float fusion_weight;
    float converge_threshold;

    // System flags
    bool enable_data_validation;    // Enable/disable range checking
    bool enable_filtering;          // Enable/disable filtering
    bool enable_timeout_check;      // Enable/disable timeout detection
} VisionConfig;

/**
 * @brief Get current vision configuration
 * @return Pointer to configuration structure
 */
VisionConfig* VisionConfig_Get(void);

/**
 * @brief Set vision configuration parameters
 * @param config New configuration to apply
 */
void VisionConfig_Set(const VisionConfig* config);

/**
 * @brief Reset configuration to default values
 */
void VisionConfig_Reset(void);

/**
 * @brief Load configuration from persistent storage (future feature)
 * @return true if loaded successfully, false otherwise
 */
bool VisionConfig_Load(void);

/**
 * @brief Save configuration to persistent storage (future feature)
 * @return true if saved successfully, false otherwise
 */
bool VisionConfig_Save(void);

#endif // VISION_CONFIG_H
