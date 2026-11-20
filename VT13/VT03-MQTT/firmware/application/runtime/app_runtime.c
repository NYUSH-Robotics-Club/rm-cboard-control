/*
 * Initializes and steps optional features without dynamic allocation. A failed
 * module is excluded from later steps and asked to enter its safe state.
 */
#include "app_runtime.h"
#include "app_manifest.h"
#include <stdbool.h>
#include <stddef.h>

#define APP_RUNTIME_MAX_MODULES (16U)

static bool s_initialized = false;
static bool s_module_ready[APP_RUNTIME_MAX_MODULES];

RobotStatus AppRuntime_Init(void) {
    if (s_initialized) {
        return ROBOT_STATUS_OK;
    }

    size_t count = 0U;
    const AppModule *modules = AppManifest_GetOptionalModules(&count);
    if (count > APP_RUNTIME_MAX_MODULES || (count > 0U && !modules)) {
        return ROBOT_STATUS_CAPACITY_EXCEEDED;
    }

    RobotStatus result = ROBOT_STATUS_OK;
    for (size_t index = 0U; index < count; ++index) {
        RobotStatus status = modules[index].init
                                 ? modules[index].init()
                                 : ROBOT_STATUS_OK;
        s_module_ready[index] = (status == ROBOT_STATUS_OK);
        if (!s_module_ready[index]) {
            if (modules[index].safe_stop) {
                modules[index].safe_stop();
            }
            result = status;
        }
    }

    s_initialized = true;
    return result;
}

void AppRuntime_Step(uint32_t now_ms) {
    if (!s_initialized) {
        return;
    }

    size_t count = 0U;
    const AppModule *modules = AppManifest_GetOptionalModules(&count);
    if (!modules || count > APP_RUNTIME_MAX_MODULES) {
        return;
    }
    for (size_t index = 0U; index < count; ++index) {
        if (s_module_ready[index] && modules[index].step) {
            modules[index].step(now_ms);
        }
    }
}

void AppRuntime_SafeStop(void) {
    size_t count = 0U;
    const AppModule *modules = AppManifest_GetOptionalModules(&count);
    if (!modules || count > APP_RUNTIME_MAX_MODULES) {
        return;
    }
    for (size_t index = 0U; index < count; ++index) {
        if (s_module_ready[index] && modules[index].safe_stop) {
            modules[index].safe_stop();
        }
        s_module_ready[index] = false;
    }
}
