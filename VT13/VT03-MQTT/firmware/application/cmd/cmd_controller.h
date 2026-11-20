/*
 * 声明中央命令控制器。
 * 任务函数接收毫秒时间戳，并把最新命令发布到消息中心。
 */
#ifndef CMD_CONTROLLER_H
#define CMD_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize command controller
 */
void CmdController_Init(void);

/**
 * @brief Process remote control input and publish commands
 * @param current_tick Current timestamp in ms
 */
void CmdController_Task(uint32_t current_tick);

#ifdef __cplusplus
}
#endif

#endif // CMD_CONTROLLER_H
