/**
 * @file vision_config.c
 * @brief Store vision tuning values for the optional enhanced receive path
 * @note Flash save/load is not implemented, and the active USB receive path
 *       does not yet use the filtering and validation settings.
 */

#include "vision_config.h"
#include <string.h>

// Global configuration instance
static VisionConfig g_vision_config = {
    // Data validation defaults
    .yaw_min = VISION_YAW_MIN,
    .yaw_max = VISION_YAW_MAX,
    .pitch_min = VISION_PITCH_MIN,
    .pitch_max = VISION_PITCH_MAX,
    .data_timeout_ms = VISION_DATA_TIMEOUT_MS,

    // Communication defaults
    .send_interval_ms = VISION_SEND_INTERVAL_MS,
    .max_retries = VISION_MAX_RETRIES,
    .retry_interval_ms = VISION_RETRY_INTERVAL_MS,

    // Filtering defaults
    .filter_window = VISION_FILTER_WINDOW,
    .ema_alpha = VISION_EMA_ALPHA,

    // Data fusion defaults
    .fusion_weight = VISION_FUSION_WEIGHT,
    .converge_threshold = VISION_CONVERGE_THRESHOLD,

    // System flags
    .enable_data_validation = true,
    .enable_filtering = true,
    .enable_timeout_check = true,
};

/**
 * @brief Get current vision configuration
 */
VisionConfig* VisionConfig_Get(void)
{
    return &g_vision_config;
}

/**
 * @brief Set vision configuration parameters
 */
void VisionConfig_Set(const VisionConfig* config)
{
    if (config != NULL) {
        memcpy(&g_vision_config, config, sizeof(VisionConfig));
    }
}

/**
 * @brief Reset configuration to default values
 */
void VisionConfig_Reset(void)
{
    g_vision_config.yaw_min = VISION_YAW_MIN;
    g_vision_config.yaw_max = VISION_YAW_MAX;
    g_vision_config.pitch_min = VISION_PITCH_MIN;
    g_vision_config.pitch_max = VISION_PITCH_MAX;
    g_vision_config.data_timeout_ms = VISION_DATA_TIMEOUT_MS;

    g_vision_config.send_interval_ms = VISION_SEND_INTERVAL_MS;
    g_vision_config.max_retries = VISION_MAX_RETRIES;
    g_vision_config.retry_interval_ms = VISION_RETRY_INTERVAL_MS;

    g_vision_config.filter_window = VISION_FILTER_WINDOW;
    g_vision_config.ema_alpha = VISION_EMA_ALPHA;

    g_vision_config.fusion_weight = VISION_FUSION_WEIGHT;
    g_vision_config.converge_threshold = VISION_CONVERGE_THRESHOLD;

    g_vision_config.enable_data_validation = true;
    g_vision_config.enable_filtering = true;
    g_vision_config.enable_timeout_check = true;
}

/**
 * @brief Load configuration from persistent storage
 * @note Future feature - currently returns false
 */
bool VisionConfig_Load(void)
{
    // TODO: Implement loading from EEPROM/Flash
    // For now, just use defaults
    return false;
}

/**
 * @brief Save configuration to persistent storage
 * @note Future feature - currently returns false
 */
bool VisionConfig_Save(void)
{
    // TODO: Implement saving to EEPROM/Flash
    return false;
}
