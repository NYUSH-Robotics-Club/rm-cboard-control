/*
 * 声明现有 USB CDC 视觉通信和尚未完全接入的诊断接口。
 * 基础接收与增强接收是两条不同路径，调用前必须确认实际接线。
 */
#ifndef VISION_COMM_H
#define VISION_COMM_H

#include <stdint.h>
#include <stdbool.h>

// Vision communication now uses USB CDC instead of UART
// #define VISION_UART_HANDLE huart6  // Old UART method

#define VISION_RECV_SIZE 18u
#define VISION_SEND_SIZE 36u  // Buffer capacity; the current 3-float frame uses 22 bytes

#pragma pack(1)

// Fire mode
typedef enum
{
    NO_FIRE = 0,
    AUTO_FIRE = 1,
    AUTO_AIM = 2
} Fire_Mode_e;

// Target state
typedef enum
{
    NO_TARGET = 0,
    TARGET_CONVERGING = 1,
    READY_TO_FIRE = 2
} Target_State_e;

// Target type
typedef enum
{
    NO_TARGET_NUM = 0,
    HERO1 = 1,
    ENGINEER2 = 2,
    INFANTRY3 = 3,
    INFANTRY4 = 4,
    INFANTRY5 = 5,
    OUTPOST = 6,
    SENTRY = 7,
    BASE = 8
} Target_Type_e;

// Vision receive data structure
typedef struct
{
    Fire_Mode_e fire_mode;
    Target_State_e target_state;
    Target_Type_e target_type;
    float pitch;
    float yaw;
    uint8_t updated;
} Vision_Recv_s;

// Enemy color
typedef enum
{
    COLOR_NONE = 0,
    COLOR_BLUE = 1,
    COLOR_RED = 2,
} Enemy_Color_e;

// Work mode
typedef enum
{
    VISION_MODE_AIM = 0,
    VISION_MODE_SMALL_BUFF = 1,
    VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

// Bullet speed
typedef enum
{
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10 = 10,
    SMALL_AMU_15 = 15,
    BIG_AMU_16 = 16,
    SMALL_AMU_18 = 18,
    SMALL_AMU_30 = 30,
} Bullet_Speed_e;

// Vision send data structure
typedef struct
{
    Enemy_Color_e enemy_color;
    Work_Mode_e work_mode;
    Bullet_Speed_e bullet_speed;
    float yaw;
    float pitch;
    float roll;
} Vision_Send_s;

#pragma pack()

/**
 * @brief Initialize vision communication module (using USB CDC)
 * @return Pointer to receive data structure
 */
Vision_Recv_s *VisionComm_Init(void);

/**
 * @brief Send vision data
 */
void VisionComm_Send(void);

/**
 * @brief Store vision send flags
 * @note The current VisionComm_Send implementation still packs fixed flags;
 *       these stored values are not used until that compatibility debt is fixed.
 * @param enemy_color Enemy color
 * @param work_mode Work mode
 * @param bullet_speed Bullet speed
 */
void VisionComm_SetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief Set attitude data for sending
 * @param yaw Yaw angle
 * @param pitch Pitch angle
 * @param roll Roll angle
 */
void VisionComm_SetAltitude(float yaw, float pitch, float roll);

/**
 * @brief Get vision receive data
 * @return Pointer to receive data structure
 */
Vision_Recv_s *VisionComm_GetData(void);

/**
 * @brief Process one frame received by the current USB CDC callback
 * @param buf Receive buffer
 * @param len Receive data length
 */
void VisionComm_RxCallback(uint8_t *buf, uint32_t len);

/**
 * @brief Compatibility no-op; USB CDC reception starts in the USB stack
 */
void VisionComm_StartReceive(void);

// =============================================================================
// Diagnostic and Monitoring Features (诊断和监控功能)
// =============================================================================

/**
 * @brief Communication diagnostics structure
 */
typedef struct {
    // Packet statistics - 数据包统计
    uint32_t total_received;        // Total packets received
    uint32_t total_sent;            // Total packets sent
    uint32_t crc_errors;            // CRC check failures
    uint32_t timeout_count;         // Data timeout occurrences
    uint32_t out_of_range_count;    // Out-of-range angle data count

    // Communication quality - 通信质量
    float packet_loss_rate;         // Packet loss rate (0.0-1.0)
    uint32_t avg_latency_ms;        // Average communication latency
    uint32_t max_latency_ms;        // Maximum latency recorded

    // Data quality - 数据质量
    float data_noise_level;         // Noise level in received data
    uint32_t last_recv_time_ms;     // Last successful receive timestamp
    bool is_data_stale;             // Data freshness flag

    // Protocol errors - 协议错误
    uint32_t length_errors;         // Incorrect packet length count
    uint32_t cmd_id_errors;         // Unknown command ID count
} VisionDiagnostics;

/**
 * @brief Data validation result
 */
typedef enum {
    VALIDATION_OK = 0,              // Data is valid
    VALIDATION_OUT_OF_RANGE,        // Angle out of valid range
    VALIDATION_TIMEOUT,             // Data is stale (timeout)
    VALIDATION_CRC_ERROR,           // CRC check failed
    VALIDATION_LENGTH_ERROR,        // Packet length incorrect
    VALIDATION_CMD_ID_ERROR         // Command ID unknown
} ValidationResult;

/**
 * @brief Filtered vision data with quality metrics
 */
typedef struct {
    float pitch_filtered;           // Filtered pitch angle
    float yaw_filtered;             // Filtered yaw angle
    float pitch_raw;                // Raw pitch angle (before filtering)
    float yaw_raw;                  // Raw yaw angle (before filtering)
    float noise_estimate;           // Estimated noise level
    uint32_t timestamp_ms;          // Receive timestamp
    ValidationResult validation;    // Validation result
} VisionDataQuality;

/**
 * @brief Control signal diagnostics
 */
typedef struct {
    // PID output monitoring - PID输出监控
    float pid_output_pitch;         // Current pitch PID output
    float pid_output_yaw;           // Current yaw PID output
    float integral_pitch;           // Pitch integral term
    float integral_yaw;             // Yaw integral term

    // Saturation detection - 饱和检测
    bool integral_saturated;        // Integral saturation flag
    bool output_saturated;          // Output saturation flag
    uint32_t saturation_count;      // Number of saturation occurrences

    // Response analysis - 响应分析
    float angle_error_pitch;        // Current pitch error
    float angle_error_yaw;          // Current yaw error
    float deadzone_threshold;       // Current deadzone threshold
    bool in_deadzone;               // Whether error is in deadzone
} ControlDiagnostics;

/**
 * @brief Get communication diagnostics
 * @return Pointer to diagnostics structure
 */
VisionDiagnostics* VisionComm_GetDiagnostics(void);

/**
 * @brief Get filtered data with quality metrics
 * @return Pointer to quality data structure
 */
VisionDataQuality* VisionComm_GetDataQuality(void);

/**
 * @brief Get control diagnostics
 * @return Pointer to control diagnostics structure
 */
ControlDiagnostics* VisionComm_GetControlDiagnostics(void);

/**
 * @brief Reset all diagnostic counters
 */
void VisionComm_ResetDiagnostics(void);

/**
 * @brief Validate received angle data
 * @param pitch Pitch angle to validate
 * @param yaw Yaw angle to validate
 * @return Validation result
 */
ValidationResult VisionComm_ValidateData(float pitch, float yaw);

/**
 * @brief Update control diagnostics (should be called from gimbal controller)
 * @param pitch_pid_output Pitch PID output
 * @param yaw_pid_output Yaw PID output
 * @param pitch_error Pitch angle error
 * @param yaw_error Yaw angle error
 */
void VisionComm_UpdateControlDiag(float pitch_pid_output, float yaw_pid_output,
                                   float pitch_error, float yaw_error);

/**
 * @brief Print diagnostic report to USB CDC
 */
void VisionComm_PrintDiagnostics(void);

#endif // VISION_COMM_H
