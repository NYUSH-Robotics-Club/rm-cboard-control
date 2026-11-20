/*
 * 定义上层模块共用的返回状态。
 * 调用方必须区分参数错误、未就绪、未实现和容量不足。
 */
#ifndef ROBOT_STATUS_H
#define ROBOT_STATUS_H

typedef enum {
    ROBOT_STATUS_OK = 0,
    ROBOT_STATUS_INVALID_ARGUMENT = -1,
    ROBOT_STATUS_NOT_FOUND = -2,
    ROBOT_STATUS_NOT_READY = -3,
    ROBOT_STATUS_UNSUPPORTED = -4,
    ROBOT_STATUS_MODE_MISMATCH = -5,
    ROBOT_STATUS_CAPACITY_EXCEEDED = -6,
    ROBOT_STATUS_IO_ERROR = -7
} RobotStatus;

#endif
