/*
 * 定义原始 CAN 接收帧并引入统一电机反馈消息。
 * 电机消息定义放在 core/contracts，避免应用依赖 CAN 模块。
 */
#ifndef CAN_COMM_H
#define CAN_COMM_H

#include <stdint.h>
#include "can_messages.h"
#include "motor_messages.h"

#endif // CAN_COMM_H
