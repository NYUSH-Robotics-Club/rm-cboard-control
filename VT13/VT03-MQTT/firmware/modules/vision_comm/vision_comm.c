/**
 * @file vision_comm.c
 * @brief Vision communication module - using USB CDC
 * @note The USB callback currently uses the basic receive path. The enhanced
 *       validation/filter path exists below but is not connected to USB yet.
 */

#include "vision_comm.h"
#include "vision_config.h"
#include "seasky_protocol.h"
#include "message_center.h"
#include "gyro_data.h"
#include "bsp_time.h"
#include "bsp_usb.h"
#include "logger.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;

// USB CDC receive processing buffer
static uint8_t cdc_recv_processing[VISION_RECV_SIZE];

// Vision send frequency control (100Hz = 10ms interval)
#define VISION_SEND_INTERVAL_MS 10
static uint32_t last_send_time = 0;

// Diagnostic and quality monitoring
static VisionDiagnostics g_diag;
static VisionDataQuality g_data_quality;
static ControlDiagnostics g_ctrl_diag;

// Moving average filter buffers
#define MAX_FILTER_SIZE 10
static float pitch_history[MAX_FILTER_SIZE];
static float yaw_history[MAX_FILTER_SIZE];
static uint8_t filter_index = 0;
static uint8_t filter_count = 0;

// EMA filter state
static float pitch_ema = 0.0f;
static float yaw_ema = 0.0f;
static bool ema_initialized = false;

/**
 * @brief USB CDC receive callback function (called from CDC_Receive_FS)
 */
void VisionComm_RxCallback(uint8_t *buf, uint32_t len)
{
    uint16_t flag_register;

    // Copy data to processing buffer
    if (len >= 18 && len <= VISION_RECV_SIZE) {
        memcpy(cdc_recv_processing, buf, len);

        // Parse protocol
        uint16_t cmd_id = get_protocol_info(cdc_recv_processing,
                                            &flag_register,
                                            (uint8_t *)&recv_data.pitch);

        if (cmd_id == 0x0001) {
            // Parse flags
            recv_data.fire_mode = (Fire_Mode_e)(flag_register & 0x03);
            recv_data.target_state = (Target_State_e)((flag_register >> 2) & 0x03);
            recv_data.target_type = (Target_Type_e)((flag_register >> 4) & 0x0F);

            // Mark data as updated
            recv_data.updated = 1;

            // Publish vision data to message center
            (void)MsgCenter_Publish(TOPIC_VISION_DATA, &recv_data, sizeof(Vision_Recv_s));
        }
    }

    // Note: USB CDC reception is handled automatically by the USB stack
    // No need to manually restart reception like with UART
}

/**
 * @brief Start USB CDC reception for vision communication
 * @note USB CDC reception is handled automatically by the USB stack.
 *       This function is kept for compatibility but does nothing.
 */
void VisionComm_StartReceive(void)
{
    // USB CDC reception is handled automatically by the USB stack
    // No action needed - the CDC_Receive_FS callback will be called automatically
}

/**
 * @brief IMU update callback - sends vision data at controlled frequency
 */
static void on_imu_update(const MsgEvent *ev, void *user_data)
{
    (void)user_data;
    
    if (ev->size == sizeof(SensorData)) {
        const SensorData *sensor_data = (const SensorData *)ev->data;
        uint32_t current_time = BspTime_NowMs();

        // Control send frequency (100Hz)
        if (current_time - last_send_time >= VISION_SEND_INTERVAL_MS) {
            // Set attitude data from gimbal IMU
            VisionComm_SetAltitude(sensor_data->g_gz, sensor_data->g_gx, sensor_data->g_gy);
            VisionComm_SetFlag(COLOR_BLUE, VISION_MODE_AIM, SMALL_AMU_15);
            VisionComm_Send();

            last_send_time = current_time;
        }
    }
}

/**
 * @brief Initialize vision communication
 */
Vision_Recv_s *VisionComm_Init(void)
{
    // Clear receive and send data
    memset(&recv_data, 0, sizeof(Vision_Recv_s));
    memset(&send_data, 0, sizeof(Vision_Send_s));
    memset(cdc_recv_processing, 0, sizeof(cdc_recv_processing));

    // Initialize diagnostics
    memset(&g_diag, 0, sizeof(VisionDiagnostics));
    memset(&g_data_quality, 0, sizeof(VisionDataQuality));
    memset(&g_ctrl_diag, 0, sizeof(ControlDiagnostics));

    // Initialize filter buffers
    memset(pitch_history, 0, sizeof(pitch_history));
    memset(yaw_history, 0, sizeof(yaw_history));
    filter_index = 0;
    filter_count = 0;
    ema_initialized = false;

    // Initialize configuration to defaults
    VisionConfig_Reset();

    // Subscribe to IMU updates for sending vision data
    (void)MsgCenter_Subscribe(TOPIC_IMU_UPDATE, on_imu_update, NULL);

    LOG_INFO(LOG_TAG_VIS, "Vision communication initialized with diagnostics");

    // Note: USB CDC reception is handled automatically by the USB stack
    // No need to manually start reception

    return &recv_data;
}

/**
 * @brief Set flags
 */
void VisionComm_SetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

/**
 * @brief Set attitude data
 */
void VisionComm_SetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief Send vision data
 */
void VisionComm_Send(void)
{
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;

    // Set flag register (example)
    flag_register = 30 << 8 | 0b00000001;

    // Convert data to seasky protocol packet
    get_protocol_send_data(0x02,                // cmd_id = 0x0002 (attitude data)
                          flag_register,
                          &send_data.yaw,       // 3 floats: yaw, pitch, roll
                          3,
                          send_buff,
                          &tx_len);

    // Send via USB CDC
    // Update send statistics only when USB accepted the frame.
    if (BspUsb_Write(send_buff, tx_len)) {
        g_diag.total_sent++;
    }
}

/**
 * @brief Get receive data
 */
Vision_Recv_s *VisionComm_GetData(void)
{
    return &recv_data;
}

// =============================================================================
// Diagnostic and Monitoring Implementation (诊断和监控功能实现)
// =============================================================================

/**
 * @brief Apply moving average filter
 */
static void apply_moving_average(float pitch, float yaw, float *pitch_out, float *yaw_out)
{
    VisionConfig *cfg = VisionConfig_Get();
    uint8_t window = (cfg->filter_window > MAX_FILTER_SIZE) ? MAX_FILTER_SIZE : cfg->filter_window;

    // Add new data to circular buffer
    pitch_history[filter_index] = pitch;
    yaw_history[filter_index] = yaw;
    filter_index = (filter_index + 1) % window;
    if (filter_count < window) {
        filter_count++;
    }

    // Calculate average
    float pitch_sum = 0.0f, yaw_sum = 0.0f;
    for (uint8_t i = 0; i < filter_count; i++) {
        pitch_sum += pitch_history[i];
        yaw_sum += yaw_history[i];
    }

    *pitch_out = pitch_sum / filter_count;
    *yaw_out = yaw_sum / filter_count;
}

/**
 * @brief Apply exponential moving average filter
 */
static void apply_ema(float pitch, float yaw, float *pitch_out, float *yaw_out)
{
    VisionConfig *cfg = VisionConfig_Get();
    float alpha = cfg->ema_alpha;

    if (!ema_initialized) {
        pitch_ema = pitch;
        yaw_ema = yaw;
        ema_initialized = true;
    } else {
        pitch_ema = alpha * pitch + (1.0f - alpha) * pitch_ema;
        yaw_ema = alpha * yaw + (1.0f - alpha) * yaw_ema;
    }

    *pitch_out = pitch_ema;
    *yaw_out = yaw_ema;
}

/**
 * @brief Calculate noise level estimate
 */
static float calculate_noise_level(void)
{
    if (filter_count < 2) {
        return 0.0f;
    }

    // Calculate standard deviation as noise estimate
    float pitch_mean = 0.0f, yaw_mean = 0.0f;
    for (uint8_t i = 0; i < filter_count; i++) {
        pitch_mean += pitch_history[i];
        yaw_mean += yaw_history[i];
    }
    pitch_mean /= filter_count;
    yaw_mean /= filter_count;

    float pitch_variance = 0.0f, yaw_variance = 0.0f;
    for (uint8_t i = 0; i < filter_count; i++) {
        float pitch_diff = pitch_history[i] - pitch_mean;
        float yaw_diff = yaw_history[i] - yaw_mean;
        pitch_variance += pitch_diff * pitch_diff;
        yaw_variance += yaw_diff * yaw_diff;
    }

    float pitch_std = sqrtf(pitch_variance / filter_count);
    float yaw_std = sqrtf(yaw_variance / filter_count);

    // Return combined noise level (Euclidean norm)
    return sqrtf(pitch_std * pitch_std + yaw_std * yaw_std);
}

/**
 * @brief Validate received angle data
 */
ValidationResult VisionComm_ValidateData(float pitch, float yaw)
{
    VisionConfig *cfg = VisionConfig_Get();

    if (!cfg->enable_data_validation) {
        return VALIDATION_OK;
    }

    // Check angle range
    if (pitch < cfg->pitch_min || pitch > cfg->pitch_max ||
        yaw < cfg->yaw_min || yaw > cfg->yaw_max) {
        g_diag.out_of_range_count++;
        LOG_WARN(LOG_TAG_VIS, "Data out of range: P=%.3f Y=%.3f", pitch, yaw);
        return VALIDATION_OUT_OF_RANGE;
    }

    // Check timeout
    if (cfg->enable_timeout_check) {
        uint32_t current_time = BspTime_NowMs();
        uint32_t elapsed = current_time - g_diag.last_recv_time_ms;
        if (elapsed > cfg->data_timeout_ms && g_diag.total_received > 0) {
            g_diag.timeout_count++;
            g_diag.is_data_stale = true;
            LOG_WARN(LOG_TAG_VIS, "Data timeout: %lu ms", elapsed);
            return VALIDATION_TIMEOUT;
        }
    }

    return VALIDATION_OK;
}

/**
 * @brief Enhanced receive callback with validation and filtering
 */
void VisionComm_RxCallback_Enhanced(uint8_t *buf, uint32_t len)
{
    VisionConfig *cfg = VisionConfig_Get();
    uint16_t flag_register;
    uint32_t current_time = BspTime_NowMs();

    // Update receive statistics
    g_diag.total_received++;
    g_diag.last_recv_time_ms = current_time;
    g_diag.is_data_stale = false;

    // Validate packet length
    if (len < 18 || len > VISION_RECV_SIZE) {
        g_diag.length_errors++;
        g_data_quality.validation = VALIDATION_LENGTH_ERROR;
        LOG_ERROR(LOG_TAG_VIS, "Invalid packet length: %lu", len);
        return;
    }

    memcpy(cdc_recv_processing, buf, len);

    // Parse protocol
    uint16_t cmd_id = get_protocol_info(cdc_recv_processing,
                                        &flag_register,
                                        (uint8_t *)&recv_data.pitch);

    if (cmd_id != 0x0001) {
        g_diag.cmd_id_errors++;
        g_data_quality.validation = VALIDATION_CMD_ID_ERROR;
        LOG_ERROR(LOG_TAG_VIS, "Unknown command ID: 0x%04X", cmd_id);
        return;
    }

    // Parse flags
    recv_data.fire_mode = (Fire_Mode_e)(flag_register & 0x03);
    recv_data.target_state = (Target_State_e)((flag_register >> 2) & 0x03);
    recv_data.target_type = (Target_Type_e)((flag_register >> 4) & 0x0F);

    // Store raw data
    g_data_quality.pitch_raw = recv_data.pitch;
    g_data_quality.yaw_raw = recv_data.yaw;
    g_data_quality.timestamp_ms = current_time;

    // Validate data
    ValidationResult validation = VisionComm_ValidateData(recv_data.pitch, recv_data.yaw);
    g_data_quality.validation = validation;

    if (validation != VALIDATION_OK) {
        // Don't use invalid data
        return;
    }

    // Apply filtering if enabled
    float pitch_filtered = recv_data.pitch;
    float yaw_filtered = recv_data.yaw;

    if (cfg->enable_filtering) {
        // Apply EMA filter (primary filter)
        apply_ema(recv_data.pitch, recv_data.yaw, &pitch_filtered, &yaw_filtered);

        // Also maintain moving average for noise estimation
        float pitch_ma, yaw_ma;
        apply_moving_average(recv_data.pitch, recv_data.yaw, &pitch_ma, &yaw_ma);

        // Update noise estimate
        g_data_quality.noise_estimate = calculate_noise_level();
        g_diag.data_noise_level = g_data_quality.noise_estimate;
    }

    // Update filtered data
    g_data_quality.pitch_filtered = pitch_filtered;
    g_data_quality.yaw_filtered = yaw_filtered;

    // Update recv_data with filtered values
    recv_data.pitch = pitch_filtered;
    recv_data.yaw = yaw_filtered;
    recv_data.updated = 1;

    // Publish vision data to message center
    (void)MsgCenter_Publish(TOPIC_VISION_DATA, &recv_data, sizeof(Vision_Recv_s));

    // Log received data (rate limited by logger)
    LOG_CSV(LOG_TAG_VIS, "RX,%.3f,%.3f,%.3f,%.3f,%d,%d",
            g_data_quality.pitch_raw, g_data_quality.yaw_raw,
            pitch_filtered, yaw_filtered,
            recv_data.target_state, validation);
}

/**
 * @brief Update control diagnostics
 */
void VisionComm_UpdateControlDiag(float pitch_pid_output, float yaw_pid_output,
                                   float pitch_error, float yaw_error)
{
    VisionConfig *cfg = VisionConfig_Get();

    g_ctrl_diag.pid_output_pitch = pitch_pid_output;
    g_ctrl_diag.pid_output_yaw = yaw_pid_output;
    g_ctrl_diag.angle_error_pitch = pitch_error;
    g_ctrl_diag.angle_error_yaw = yaw_error;

    // Check if error is in deadzone
    float error_magnitude = sqrtf(pitch_error * pitch_error + yaw_error * yaw_error);
    g_ctrl_diag.in_deadzone = (error_magnitude < cfg->converge_threshold);

    // Check for output saturation (assuming max output is CURRENT_LIMIT)
    const float OUTPUT_LIMIT = 30000.0f;  // Should match CURRENT_LIMIT in gimbal_controller
    if (fabsf(pitch_pid_output) > OUTPUT_LIMIT * 0.95f ||
        fabsf(yaw_pid_output) > OUTPUT_LIMIT * 0.95f) {
        g_ctrl_diag.output_saturated = true;
        g_ctrl_diag.saturation_count++;
        LOG_WARN(LOG_TAG_VIS, "Control output saturated: P=%.0f Y=%.0f",
                 pitch_pid_output, yaw_pid_output);
    } else {
        g_ctrl_diag.output_saturated = false;
    }

    // Log control diagnostics (rate limited)
    LOG_CSV(LOG_TAG_VIS, "CTRL,%.3f,%.3f,%.0f,%.0f,%d,%d",
            pitch_error, yaw_error,
            pitch_pid_output, yaw_pid_output,
            g_ctrl_diag.in_deadzone, g_ctrl_diag.output_saturated);
}

/**
 * @brief Get diagnostics
 */
VisionDiagnostics* VisionComm_GetDiagnostics(void)
{
    // Update packet loss rate
    if (g_diag.total_sent > 0) {
        uint32_t expected_recv = g_diag.total_sent;  // Simplified assumption
        uint32_t actual_recv = g_diag.total_received;
        if (expected_recv > actual_recv) {
            g_diag.packet_loss_rate = (float)(expected_recv - actual_recv) / expected_recv;
        } else {
            g_diag.packet_loss_rate = 0.0f;
        }
    }

    return &g_diag;
}

/**
 * @brief Get data quality
 */
VisionDataQuality* VisionComm_GetDataQuality(void)
{
    return &g_data_quality;
}

/**
 * @brief Get control diagnostics
 */
ControlDiagnostics* VisionComm_GetControlDiagnostics(void)
{
    return &g_ctrl_diag;
}

/**
 * @brief Reset diagnostics
 */
void VisionComm_ResetDiagnostics(void)
{
    memset(&g_diag, 0, sizeof(VisionDiagnostics));
    memset(&g_data_quality, 0, sizeof(VisionDataQuality));
    memset(&g_ctrl_diag, 0, sizeof(ControlDiagnostics));

    // Reset filter state
    filter_index = 0;
    filter_count = 0;
    ema_initialized = false;

    LOG_INFO(LOG_TAG_VIS, "Diagnostics reset");
}

/**
 * @brief Print diagnostic report
 */
void VisionComm_PrintDiagnostics(void)
{
    char msg[256];

    snprintf(msg, sizeof(msg),
             "\n=== Vision Communication Diagnostics ===\n"
             "Packets: RX=%lu TX=%lu\n"
             "Errors: CRC=%lu Timeout=%lu Range=%lu\n"
             "Quality: Loss=%.1f%% Noise=%.4f Stale=%d\n"
             "Control: InDZ=%d Sat=%d (count=%lu)\n"
             "Angles: P=%.3f/%.3f Y=%.3f/%.3f\n",
             (unsigned long)g_diag.total_received, (unsigned long)g_diag.total_sent,
             (unsigned long)g_diag.crc_errors, (unsigned long)g_diag.timeout_count,
             (unsigned long)g_diag.out_of_range_count,
             g_diag.packet_loss_rate * 100.0f, g_diag.data_noise_level, g_diag.is_data_stale,
             g_ctrl_diag.in_deadzone, g_ctrl_diag.output_saturated,
             (unsigned long)g_ctrl_diag.saturation_count,
             g_data_quality.pitch_raw, g_data_quality.pitch_filtered,
             g_data_quality.yaw_raw, g_data_quality.yaw_filtered);

    (void)BspUsb_Write((const uint8_t *)msg, (uint16_t)strlen(msg));

    LOG_INFO(LOG_TAG_VIS, "Diagnostics printed to USB CDC");
}
