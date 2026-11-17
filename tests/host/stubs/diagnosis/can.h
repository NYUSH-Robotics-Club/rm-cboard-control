#ifndef MOCK_CAN_H
#define MOCK_CAN_H
#include <stdint.h>
#define ENABLE 1U
#define DISABLE 0U
#define HAL_OK 0U
#define HAL_CAN_STATE_READY 1U
#define HAL_CAN_STATE_LISTENING 2U
#define CAN_MODE_NORMAL 0U
#define CAN_MODE_SILENT 1U
#define CAN_MODE_SILENT_LOOPBACK 2U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 1U
#define CAN_RX_FIFO0 0U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define CAN_IT_RX_FIFO0_OVERRUN 2U
#define HAL_CAN_ERROR_RX_FOV0 4U
#define CAN_ID_STD 0U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define CAN_RTR_REMOTE 2U
#define CAN_ESR_BOFF 4U
#define CAN_TSR_RQCP0 1U
#define CAN_TSR_TXOK0 2U
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN1_RX0_IRQn 20
#define CAN2_RX0_IRQn 21
typedef struct { uint32_t ESR, TSR, RF0R; } MockRegisters;
typedef struct {
    MockRegisters *Instance;
    struct {uint32_t Prescaler, Mode, AutoRetransmission, AutoBusOff;} Init;
} CAN_HandleTypeDef;
typedef struct {
    uint32_t FilterBank, FilterMode, FilterScale, FilterFIFOAssignment;
    uint32_t FilterActivation, SlaveStartFilterBank;
} CAN_FilterTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef CAN_TxHeaderTypeDef CAN_RxHeaderTypeDef;
extern CAN_HandleTypeDef hcan1, hcan2;
uint32_t HAL_CAN_GetState(CAN_HandleTypeDef *h);
uint32_t HAL_CAN_Stop(CAN_HandleTypeDef *h);
uint32_t HAL_CAN_Init(CAN_HandleTypeDef *h);
uint32_t HAL_CAN_Start(CAN_HandleTypeDef *h);
uint32_t HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f);
uint32_t HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t flags);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo);
uint32_t HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo, CAN_RxHeaderTypeDef *header, uint8_t *data);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *h);
uint32_t HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *header, uint8_t *data, uint32_t *mailbox);
uint32_t HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t mailbox);
void HAL_NVIC_DisableIRQ(int irq);
void HAL_NVIC_EnableIRQ(int irq);
uint32_t HAL_GetTick(void);
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
static inline void __DMB(void) { __atomic_thread_fence(__ATOMIC_SEQ_CST); }
#endif
