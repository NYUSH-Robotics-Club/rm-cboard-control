/*
 * Exposes the one list where optional application features are registered.
 * Hardware startup order is intentionally not described by this manifest.
 */
#ifndef APP_MANIFEST_H
#define APP_MANIFEST_H

#include <stddef.h>
#include "app_runtime.h"

const AppModule *AppManifest_GetOptionalModules(size_t *count);

#endif
