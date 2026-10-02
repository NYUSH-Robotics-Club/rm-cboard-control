/* Minimal hardware model for the real BSP recovery state machine. */
#ifndef TEST_CAN_RECOVERY_MAIN_H
#define TEST_CAN_RECOVERY_MAIN_H
#include <stdint.h>
typedef struct { volatile uint32_t MCR, MSR, ESR, TSR; uint32_t free_slots; } CAN_TypeDef;
typedef struct { CAN_TypeDef *Instance; } CAN_HandleTypeDef;
typedef struct {
 uint32_t FilterBank, SlaveStartFilterBank, FilterActivation, FilterMode, FilterScale;
 uint32_t FilterFIFOAssignment, FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow;
} CAN_FilterTypeDef;
typedef struct { uint32_t StdId, IDE, RTR, DLC; } CAN_TxHeaderTypeDef;
typedef CAN_TxHeaderTypeDef CAN_RxHeaderTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define ENABLE 1U
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 1U
#define CAN_FILTER_FIFO0 0U
#define CAN_RX_FIFO0 0U
#define CAN_IT_RX_FIFO0_MSG_PENDING 2U
#define CAN_ID_STD 0U
#define CAN_RTR_DATA 0U
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN_MCR_INRQ 1U
#define CAN_MSR_INAK 1U
#define CAN_ESR_EWGF 1U
#define CAN_ESR_EPVF 2U
#define CAN_ESR_BOFF 4U
#define CAN_ESR_LEC 0x70U
#define SET_BIT(r,b) ((r)|=(b))
#define CLEAR_BIT(r,b) ((r)&=~(b))
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *, CAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, CAN_TxHeaderTypeDef *, uint8_t *, uint32_t *);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *, uint32_t);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *);
/* The diagnostic method also reads these names; the model has no FIFO. */
#define RF0R free_slots
#endif
