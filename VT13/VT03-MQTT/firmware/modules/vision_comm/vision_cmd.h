/**
 * @file vision_cmd.h
 * @brief Vision system command interface for runtime parameter adjustment
 * @note Parsing functions exist, but the active USB receive callback does not
 *       route command frames here yet.
 */

#ifndef VISION_CMD_H
#define VISION_CMD_H

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// Command Interface (命令接口)
// =============================================================================

/**
 * @brief Vision system commands
 */
typedef enum {
    VCMD_GET_DIAG = 0x01,           // Get diagnostics report
    VCMD_RESET_DIAG = 0x02,         // Reset diagnostic counters
    VCMD_SET_FILTER_WINDOW = 0x10,  // Set filter window size (param: 1-10)
    VCMD_SET_EMA_ALPHA = 0x11,      // Set EMA alpha (param: 0.0-1.0)
    VCMD_SET_FUSION_WEIGHT = 0x12,  // Set data fusion weight (param: 0.0-1.0)
    VCMD_SET_SEND_INTERVAL = 0x13,  // Set send interval in ms (param: 5-50)
    VCMD_TOGGLE_VALIDATION = 0x20,  // Toggle data validation on/off
    VCMD_TOGGLE_FILTERING = 0x21,   // Toggle filtering on/off
    VCMD_TOGGLE_TIMEOUT_CHK = 0x22, // Toggle timeout check on/off
    VCMD_SET_TIMEOUT_MS = 0x23,     // Set data timeout in ms (param: 50-1000)
    VCMD_RESET_CONFIG = 0x30,       // Reset all config to defaults
    VCMD_SAVE_CONFIG = 0x31,        // Save config to flash (future feature)
    VCMD_LOAD_CONFIG = 0x32,        // Load config from flash (future feature)
} VisionCommand;

/**
 * @brief Command packet structure
 */
#pragma pack(1)
typedef struct {
    uint8_t header;                 // Fixed: 0xAA
    uint8_t cmd;                    // Command ID from VisionCommand enum
    float param;                    // Command parameter (float for flexibility)
    uint8_t checksum;               // Simple checksum: XOR of all bytes
} VisionCmdPacket;
#pragma pack()

/**
 * @brief Command response codes
 */
typedef enum {
    VCMD_RESP_OK = 0x00,            // Command executed successfully
    VCMD_RESP_INVALID_CMD = 0x01,   // Unknown command
    VCMD_RESP_INVALID_PARAM = 0x02, // Parameter out of valid range
    VCMD_RESP_NOT_IMPLEMENTED = 0x03, // Feature not implemented yet
    VCMD_RESP_ERROR = 0xFF          // General error
} VisionCmdResponse;

/**
 * @brief Initialize vision command interface
 */
void VisionCmd_Init(void);

/**
 * @brief Process received command packet
 * @param packet Command packet to process
 * @return Response code
 */
VisionCmdResponse VisionCmd_Process(const VisionCmdPacket *packet);

/**
 * @brief Parse command from buffer (called from USB CDC receive)
 * @param buf Receive buffer
 * @param len Buffer length
 * @return true if valid command packet found and processed
 */
bool VisionCmd_ParseAndExecute(uint8_t *buf, uint32_t len);

/**
 * @brief Send command response via USB CDC
 * @param cmd Original command ID
 * @param response Response code
 * @param data Optional response data (can be NULL)
 */
void VisionCmd_SendResponse(uint8_t cmd, VisionCmdResponse response, const char *data);

// =============================================================================
// Helper Functions for Text-Based Commands (文本命令辅助函数)
// =============================================================================

/**
 * @brief Parse and execute text-based command
 * @param cmd_str Command string (e.g., "set_filter_window 5")
 * @return true if command was recognized and executed
 * @note Supports human-readable commands for debugging
 */
bool VisionCmd_ParseText(const char *cmd_str);

/**
 * @brief Print available commands to USB CDC
 */
void VisionCmd_PrintHelp(void);

#endif // VISION_CMD_H
