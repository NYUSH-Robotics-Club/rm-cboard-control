/* Host-only HAL model. It checks receive ownership and recovery, not electrical timing. */
#ifndef TEST_REMOTE_MAIN_H
#define TEST_REMOTE_MAIN_H
#include <stdint.h>
typedef struct { volatile uint32_t SR, DR, CR3; } TestUart;
typedef struct { volatile uint32_t CR, NDTR; } TestDma;
typedef struct { TestDma *Instance; } DMA_HandleTypeDef;
typedef struct { TestUart *Instance; uint32_t ErrorCode; DMA_HandleTypeDef *hdmarx; uint8_t *pRxBuffPtr; uint32_t gState; uint32_t RxState; } UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
extern TestUart test_uart;
#define USART3 (&test_uart)
#define USART1 ((TestUart *)0)
#define HAL_UART_STATE_BUSY_TX 0x21U
/* VR archive host stub: no ARM memory ordering is modeled here. */
#define HAL_UART_STATE_READY 0x20U
#define HAL_UART_STATE_BUSY_RX 0x22U
#define __DMB() ((void)0)
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
#define USART3_IRQn 0
#define DMA1_Stream1_IRQn 1
#define UART_FLAG_PE 1U
#define UART_FLAG_FE 2U
#define UART_FLAG_NE 4U
#define UART_FLAG_ORE 8U
#define UART_FLAG_IDLE 16U
#define UART_IT_IDLE UART_FLAG_IDLE
#define HAL_UART_ERROR_NONE 0U
#define DMA_SxCR_EN 1U
#define DMA_SxCR_CIRC (1U << 8)
#define DMA_SxCR_DBM (1U << 18)
#define DMA_SxCR_CT (1U << 19)
#define DMA_IT_HT 16U
#define CLEAR_BIT(reg, bits) ((reg) &= ~(bits))
#define __HAL_UART_CLEAR_PEFLAG(h) ((h)->Instance->SR = 0)
#define __HAL_DMA_DISABLE_IT(h, flag) ((void)(h), (void)(flag))
#define __HAL_UART_ENABLE_IT(h, flag) ((void)(h), (void)(flag))
uint32_t NVIC_GetEnableIRQ(int irq);
void NVIC_DisableIRQ(int irq);
void NVIC_EnableIRQ(int irq);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *buffer, uint16_t length);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *h);
typedef enum { HAL_UART_RXEVENT_TC, HAL_UART_RXEVENT_HT, HAL_UART_RXEVENT_IDLE } HAL_UART_RxEventTypeTypeDef;
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *h);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
#endif
