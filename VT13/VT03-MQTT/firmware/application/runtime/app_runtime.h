/*
 * Runs optional application features from one static manifest. Existing boot
 * modules remain under the frozen main entry until that boundary is released.
 */
#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include <stdint.h>
#include "robot_status.h"

typedef struct {
    const char *name;
    RobotStatus (*init)(void);
    void (*step)(uint32_t now_ms);
    void (*safe_stop)(void);
} AppModule;

RobotStatus AppRuntime_Init(void);
void AppRuntime_Step(uint32_t now_ms);
void AppRuntime_SafeStop(void);

#endif
