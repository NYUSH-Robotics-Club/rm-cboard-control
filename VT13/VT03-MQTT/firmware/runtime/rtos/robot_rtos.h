/*
 * Starts the robot scheduler after all blocking hardware setup is complete.
 * The function returns false only when task creation or scheduler start fails.
 */
#ifndef ROBOT_RTOS_H
#define ROBOT_RTOS_H

#include <stdbool.h>

bool RobotRtos_Start(void);

#endif
