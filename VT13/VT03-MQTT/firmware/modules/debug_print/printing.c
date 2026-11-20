
/*
 * 提供通过 USB CDC 输出格式化调试文本的兼容函数。
 * 大量输出可能阻塞或丢失，控制周期内应优先使用限流 Logger。
 */
#include "printing.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_usb.h"

// Current print mode (default to USB)
static PrintMode_t current_print_mode = PRINT_MODE_USB;

/* 选择后续调试文本的输出通道。 */
void Debug_SetPrintMode(PrintMode_t mode)
{
    current_print_mode = mode;
}

/* 返回当前调试文本输出通道。 */
PrintMode_t Debug_GetPrintMode(void)
{
    return current_print_mode;
}

/* 按当前通道发送完整字符串。 */
void Debug_SendString(const char* message)
{
    if (message == NULL)
    {
        return;
    }

    if (current_print_mode == PRINT_MODE_USB)
    {
        (void)BspUsb_Write((const uint8_t *)message, (uint16_t)strlen(message));
    }
    else
    {
        (void)BspUart6_Write((const uint8_t *)message, (uint16_t)strlen(message));
    }
}

/* 格式化后按当前通道发送，单次最多使用 128 字节缓冲区。 */
void Debug_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = sizeof(buf);

    if (current_print_mode == PRINT_MODE_USB)
    {
        (void)BspUsb_Write((const uint8_t *)buf, (uint16_t)n);
    }
    else
    {
        (void)BspUart6_Write((const uint8_t *)buf, (uint16_t)n);
    }
}

/* 保留旧 USB 函数名，内部统一走 BSP。 */
#if defined(__GNUC__)
__attribute__((unused))
#endif
void USB_CDC_SendString(const char* message)
{
    if (message != NULL)
    {
        (void)BspUsb_Write((const uint8_t *)message, (uint16_t)strlen(message));
    }
}

/* 保留旧格式化函数名，内部统一走 BSP。 */
void USB_CDC_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    (void)BspUsb_Write((const uint8_t *)buf, (uint16_t)n);
}


 void Debug_PrintCANStatus(uint32_t current_tick, const CAN_Manager_t *can1_manager, const CAN_Manager_t *can2_manager)
{
  Debug_Printf("CAN1 tx_ok=%lu tx_err=%lu rx=%lu last_rx_id=0x%03lX last_tx=%lums last_rx=%lums\r\n",
      (unsigned long)CAN_Manager_GetTxOk(can1_manager),
      (unsigned long)CAN_Manager_GetTxErr(can1_manager),
      (unsigned long)CAN_Manager_GetRxFrames(can1_manager),
      (unsigned long)CAN_Manager_GetLastRxId(can1_manager),
      (unsigned long)(current_tick - CAN_Manager_GetLastTxTime(can1_manager)),
      (unsigned long)(current_tick - CAN_Manager_GetLastRxTime(can1_manager)));

  Debug_Printf("CAN2 tx_ok=%lu tx_err=%lu rx=%lu last_rx_id=0x%03lX last_tx=%lums last_rx=%lums\r\n",
      (unsigned long)CAN_Manager_GetTxOk(can2_manager),
      (unsigned long)CAN_Manager_GetTxErr(can2_manager),
      (unsigned long)CAN_Manager_GetRxFrames(can2_manager),
      (unsigned long)CAN_Manager_GetLastRxId(can2_manager),
      (unsigned long)(current_tick - CAN_Manager_GetLastTxTime(can2_manager)),
      (unsigned long)(current_tick - CAN_Manager_GetLastRxTime(can2_manager)));
}

 void Debug_PrintCANDiag(void)
{
  BspCanDiagnostics diagnostics;
  if (!BspCan_ReadDiagnostics(BSP_CAN_CHANNEL_2, &diagnostics)) {
    Debug_Printf("CAN2 diag unavailable\r\n");
    return;
  }
  Debug_Printf("CAN2 diag err=0x%08lX ESR=0x%08lX TSR=0x%08lX RF0R=0x%08lX TXMB_FREE=%lu\r\n",
    (unsigned long)diagnostics.error,
    (unsigned long)diagnostics.error_status_register,
    (unsigned long)diagnostics.transmit_status_register,
    (unsigned long)diagnostics.receive_fifo0_register,
    (unsigned long)diagnostics.free_tx_mailboxes);
}

/*
Print RC and control path diagnostics
Usage: Called periodically in main loop for debugging
*/
void Debug_PrintRCDiagnostics(uint32_t current_tick, uint32_t *last_debug_time, uint32_t *last_frame_count)
{
  if (current_tick - *last_debug_time < USB_DEBUG_INTERVAL_MS)
  {
    return;
  }

  *last_debug_time = current_tick;
  uint32_t fc = RC_GetFrameCount();
  (void)(*last_frame_count);

  // RC and control path quick diagnostics
  {
    // Print sanitized channels (after baseline removal)
    const RC_ctrl_t *raw_rc = get_remote_control_point();
    int16_t ch0 = 0, ch2 = 0, ch3 = 0, ch4 = 0; 
    uint8_t swl = 0;
    
    if (raw_rc) 
    { 
      ch0 = raw_rc->rc.ch[0]; 
      ch2 = raw_rc->rc.ch[2]; 
      ch3 = raw_rc->rc.ch[3]; 
      ch4 = raw_rc->rc.ch[4]; 
      swl = (uint8_t)raw_rc->rc.s[0]; 
    }
    Debug_Printf("RC fc=%lu ch0=%d ch2=%d ch3=%d ch4=%d swL=%u\r\n",
      (unsigned long)fc, (int)ch0, (int)ch2, (int)ch3, (int)ch4, (unsigned int)swl);
  }
  
  *last_frame_count = fc;
}
