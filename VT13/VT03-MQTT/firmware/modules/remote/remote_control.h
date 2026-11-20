/*
 * 解析 DR16 遥控器的 18 字节 SBUS 帧。
 * 应用使用的数据结构在 core/contracts 中，本文件只声明设备接口。
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include <stdint.h>
#include "remote_messages.h"

#define RC_FRAME_LENGTH 18u

/**
  * @brief          remote control initialization
  * @param[in]      none
  * @retval         none
  */
extern void remote_control_init(void);
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
extern const RC_ctrl_t *get_remote_control_point(void);
uint32_t RC_GetFrameCount(void);
// Debug helper: copy last raw SBUS frame (18 bytes)
void RC_GetLastFrame(uint8_t out[RC_FRAME_LENGTH]);

#endif
