/**
 * @file vision_cmd.c
 * @brief Parse optional vision tuning commands
 * @note This file is compiled, but the active USB callback does not call it yet.
 */

#include "vision_cmd.h"
#include "vision_config.h"
#include "vision_comm.h"
#include "bsp_usb.h"
#include "logger.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define CMD_HEADER 0xAA

/**
 * @brief Calculate checksum for command packet
 */
static uint8_t calculate_checksum(const VisionCmdPacket *packet)
{
    uint8_t *bytes = (uint8_t *)packet;
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(VisionCmdPacket) - 1; i++) {
        checksum ^= bytes[i];
    }
    return checksum;
}

/**
 * @brief Initialize vision command interface
 */
void VisionCmd_Init(void)
{
    LOG_INFO(LOG_TAG_VIS, "Vision command interface initialized");
}

/**
 * @brief Process received command packet
 */
VisionCmdResponse VisionCmd_Process(const VisionCmdPacket *packet)
{
    VisionConfig *cfg = VisionConfig_Get();

    switch (packet->cmd) {
        case VCMD_GET_DIAG:
            VisionComm_PrintDiagnostics();
            return VCMD_RESP_OK;

        case VCMD_RESET_DIAG:
            VisionComm_ResetDiagnostics();
            return VCMD_RESP_OK;

        case VCMD_SET_FILTER_WINDOW: {
            uint8_t window = (uint8_t)packet->param;
            if (window < 1 || window > 10) {
                return VCMD_RESP_INVALID_PARAM;
            }
            cfg->filter_window = window;
            LOG_INFO(LOG_TAG_VIS, "Filter window set to %d", window);
            return VCMD_RESP_OK;
        }

        case VCMD_SET_EMA_ALPHA: {
            float alpha = packet->param;
            if (alpha < 0.0f || alpha > 1.0f) {
                return VCMD_RESP_INVALID_PARAM;
            }
            cfg->ema_alpha = alpha;
            LOG_INFO(LOG_TAG_VIS, "EMA alpha set to %.3f", alpha);
            return VCMD_RESP_OK;
        }

        case VCMD_SET_FUSION_WEIGHT: {
            float weight = packet->param;
            if (weight < 0.0f || weight > 1.0f) {
                return VCMD_RESP_INVALID_PARAM;
            }
            cfg->fusion_weight = weight;
            LOG_INFO(LOG_TAG_VIS, "Fusion weight set to %.3f", weight);
            return VCMD_RESP_OK;
        }

        case VCMD_SET_SEND_INTERVAL: {
            uint32_t interval = (uint32_t)packet->param;
            if (interval < 5 || interval > 50) {
                return VCMD_RESP_INVALID_PARAM;
            }
            cfg->send_interval_ms = interval;
            LOG_INFO(LOG_TAG_VIS, "Send interval set to %lu ms", interval);
            return VCMD_RESP_OK;
        }

        case VCMD_TOGGLE_VALIDATION:
            cfg->enable_data_validation = !cfg->enable_data_validation;
            LOG_INFO(LOG_TAG_VIS, "Data validation %s",
                     cfg->enable_data_validation ? "enabled" : "disabled");
            return VCMD_RESP_OK;

        case VCMD_TOGGLE_FILTERING:
            cfg->enable_filtering = !cfg->enable_filtering;
            LOG_INFO(LOG_TAG_VIS, "Filtering %s",
                     cfg->enable_filtering ? "enabled" : "disabled");
            return VCMD_RESP_OK;

        case VCMD_TOGGLE_TIMEOUT_CHK:
            cfg->enable_timeout_check = !cfg->enable_timeout_check;
            LOG_INFO(LOG_TAG_VIS, "Timeout check %s",
                     cfg->enable_timeout_check ? "enabled" : "disabled");
            return VCMD_RESP_OK;

        case VCMD_SET_TIMEOUT_MS: {
            uint32_t timeout = (uint32_t)packet->param;
            if (timeout < 50 || timeout > 1000) {
                return VCMD_RESP_INVALID_PARAM;
            }
            cfg->data_timeout_ms = timeout;
            LOG_INFO(LOG_TAG_VIS, "Data timeout set to %lu ms", timeout);
            return VCMD_RESP_OK;
        }

        case VCMD_RESET_CONFIG:
            VisionConfig_Reset();
            LOG_INFO(LOG_TAG_VIS, "Configuration reset to defaults");
            return VCMD_RESP_OK;

        case VCMD_SAVE_CONFIG:
        case VCMD_LOAD_CONFIG:
            LOG_WARN(LOG_TAG_VIS, "Config save/load not implemented yet");
            return VCMD_RESP_NOT_IMPLEMENTED;

        default:
            LOG_ERROR(LOG_TAG_VIS, "Unknown command: 0x%02X", packet->cmd);
            return VCMD_RESP_INVALID_CMD;
    }
}

/**
 * @brief Parse command from buffer
 */
bool VisionCmd_ParseAndExecute(uint8_t *buf, uint32_t len)
{
    if (len < sizeof(VisionCmdPacket)) {
        return false;
    }

    VisionCmdPacket *packet = (VisionCmdPacket *)buf;

    // Verify header
    if (packet->header != CMD_HEADER) {
        return false;
    }

    // Verify checksum
    uint8_t expected_checksum = calculate_checksum(packet);
    if (packet->checksum != expected_checksum) {
        LOG_ERROR(LOG_TAG_VIS, "Command checksum error");
        VisionCmd_SendResponse(packet->cmd, VCMD_RESP_ERROR, "Checksum mismatch");
        return true;  // We recognized it as a command attempt
    }

    // Process command
    VisionCmdResponse response = VisionCmd_Process(packet);
    VisionCmd_SendResponse(packet->cmd, response, NULL);

    return true;
}

/**
 * @brief Send command response
 */
void VisionCmd_SendResponse(uint8_t cmd, VisionCmdResponse response, const char *data)
{
    char msg[128];
    int len;

    if (data != NULL) {
        len = snprintf(msg, sizeof(msg), "[VCMD] Cmd=0x%02X Response=%d: %s\n",
                       cmd, response, data);
    } else {
        const char *resp_str;
        switch (response) {
            case VCMD_RESP_OK: resp_str = "OK"; break;
            case VCMD_RESP_INVALID_CMD: resp_str = "Invalid command"; break;
            case VCMD_RESP_INVALID_PARAM: resp_str = "Invalid parameter"; break;
            case VCMD_RESP_NOT_IMPLEMENTED: resp_str = "Not implemented"; break;
            default: resp_str = "Error"; break;
        }
        len = snprintf(msg, sizeof(msg), "[VCMD] Cmd=0x%02X %s\n", cmd, resp_str);
    }

    if (len > 0) {
        (void)BspUsb_Write((const uint8_t *)msg, (uint16_t)len);
    }
}

/**
 * @brief Parse text-based command
 */
bool VisionCmd_ParseText(const char *cmd_str)
{
    char cmd_name[32];
    float param = 0.0f;

    // Parse command string
    int parsed = sscanf(cmd_str, "%31s %f", cmd_name, &param);
    if (parsed < 1) {
        return false;
    }

    // Build command packet
    VisionCmdPacket packet;
    packet.header = CMD_HEADER;
    packet.param = param;

    // Match command name
    if (strcmp(cmd_name, "get_diag") == 0) {
        packet.cmd = VCMD_GET_DIAG;
    } else if (strcmp(cmd_name, "reset_diag") == 0) {
        packet.cmd = VCMD_RESET_DIAG;
    } else if (strcmp(cmd_name, "set_filter_window") == 0) {
        packet.cmd = VCMD_SET_FILTER_WINDOW;
    } else if (strcmp(cmd_name, "set_ema_alpha") == 0) {
        packet.cmd = VCMD_SET_EMA_ALPHA;
    } else if (strcmp(cmd_name, "set_fusion_weight") == 0) {
        packet.cmd = VCMD_SET_FUSION_WEIGHT;
    } else if (strcmp(cmd_name, "set_send_interval") == 0) {
        packet.cmd = VCMD_SET_SEND_INTERVAL;
    } else if (strcmp(cmd_name, "toggle_validation") == 0) {
        packet.cmd = VCMD_TOGGLE_VALIDATION;
    } else if (strcmp(cmd_name, "toggle_filtering") == 0) {
        packet.cmd = VCMD_TOGGLE_FILTERING;
    } else if (strcmp(cmd_name, "toggle_timeout") == 0) {
        packet.cmd = VCMD_TOGGLE_TIMEOUT_CHK;
    } else if (strcmp(cmd_name, "set_timeout_ms") == 0) {
        packet.cmd = VCMD_SET_TIMEOUT_MS;
    } else if (strcmp(cmd_name, "reset_config") == 0) {
        packet.cmd = VCMD_RESET_CONFIG;
    } else if (strcmp(cmd_name, "help") == 0) {
        VisionCmd_PrintHelp();
        return true;
    } else {
        return false;  // Unknown command
    }

    // Calculate checksum and process
    packet.checksum = calculate_checksum(&packet);
    VisionCmdResponse response = VisionCmd_Process(&packet);
    VisionCmd_SendResponse(packet.cmd, response, NULL);

    return true;
}

/**
 * @brief Print help information
 */
void VisionCmd_PrintHelp(void)
{
    const char *help_text =
        "\n=== Vision System Commands ===\n"
        "Diagnostics:\n"
        "  get_diag                 - Print diagnostic report\n"
        "  reset_diag               - Reset diagnostic counters\n"
        "\n"
        "Filter Parameters:\n"
        "  set_filter_window <1-10> - Set moving average window size\n"
        "  set_ema_alpha <0.0-1.0>  - Set EMA filter alpha\n"
        "  toggle_filtering         - Enable/disable filtering\n"
        "\n"
        "Data Fusion:\n"
        "  set_fusion_weight <0.0-1.0> - Set vision/IMU fusion weight\n"
        "\n"
        "Communication:\n"
        "  set_send_interval <5-50>  - Set send interval (ms)\n"
        "  set_timeout_ms <50-1000>  - Set data timeout (ms)\n"
        "  toggle_timeout            - Enable/disable timeout check\n"
        "\n"
        "Validation:\n"
        "  toggle_validation        - Enable/disable data validation\n"
        "\n"
        "Configuration:\n"
        "  reset_config             - Reset all settings to defaults\n"
        "  help                     - Show this help message\n"
        "\n";

    (void)BspUsb_Write((const uint8_t *)help_text, (uint16_t)strlen(help_text));
}
