/*
 * 声明 USB CDC 调试打印接口。
 * 该接口用于诊断，不应承担控制数据传输。
 */
#ifndef PRINTING_H
#define PRINTING_H

#include "can_manager.h"
#include "remote_control.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// Debug info interval
#define USB_DEBUG_INTERVAL_MS (1000U)

// Print mode selection
typedef enum {
    PRINT_MODE_USB,   // Print via USB CDC
    PRINT_MODE_UART   // Print via USART6
} PrintMode_t;

// Set the current print mode
void Debug_SetPrintMode(PrintMode_t mode);

// Get the current print mode
PrintMode_t Debug_GetPrintMode(void);

// Unified print functions (supports both USB and UART)
void Debug_SendString(const char* message);
void Debug_Printf(const char *fmt, ...);

// Legacy USB CDC functions (for backward compatibility)
void USB_CDC_SendString(const char* message);
void USB_CDC_Printf(const char *fmt, ...);

void Debug_PrintCANStatus(uint32_t current_tick, const CAN_Manager_t *can1_manager, const CAN_Manager_t *can2_manager);
void Debug_PrintCANDiag(void);
void Debug_PrintRCDiagnostics(uint32_t current_tick, uint32_t *last_debug_time, uint32_t *last_frame_count);

#endif
