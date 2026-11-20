/*
 * 提供很短代码段使用的板级保护接口，避免上层直接操作中断开关。
 * 返回值只供配对退出时恢复原状态，调用者不应解释其位含义。
 */
#ifndef BSP_CRITICAL_H
#define BSP_CRITICAL_H

#include <stdint.h>

typedef uint32_t BspCriticalState;

BspCriticalState BspCritical_Enter(void);
void BspCritical_Exit(BspCriticalState previous_state);

#endif
