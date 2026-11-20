/* Compile the unmodified upstream USART service; rename callbacks for coexistence
 * with WT61C and observe HAL return values without changing receive behavior. */
#include "main.h"
HAL_StatusTypeDef BspRc_ObservedReceive(UART_HandleTypeDef *, uint8_t *, uint16_t);
#define HAL_UARTEx_ReceiveToIdle_DMA BspRc_ObservedReceive
#define HAL_UARTEx_RxEventCallback BspRc_UpstreamRxEvent
#define HAL_UART_ErrorCallback BspRc_UpstreamError
#include "../../third_party/nyush_remote/bsp/usart/bsp_usart.c"
