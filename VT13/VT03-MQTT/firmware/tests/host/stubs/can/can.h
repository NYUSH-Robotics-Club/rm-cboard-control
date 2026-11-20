/* Only opaque CAN handles/status are faked; CAN packing runs in production code. */
#ifndef TEST_CAN_H
#define TEST_CAN_H
typedef struct { unsigned channel; } CAN_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
#endif
