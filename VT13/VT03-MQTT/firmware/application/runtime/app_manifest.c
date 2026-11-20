/*
 * Central registration list for future application features. Add only modules
 * that can start after all required controllers and run without blocking.
 */
#include "app_manifest.h"

/*
 * Keep a sentinel so the file remains valid C while no optional feature is
 * registered. The reported count is zero, so the sentinel never executes.
 */
static const AppModule s_optional_modules[] = {
    {0}
};

const AppModule *AppManifest_GetOptionalModules(size_t *count) {
    if (count) {
        *count = 0U;
    }
    return s_optional_modules;
}
