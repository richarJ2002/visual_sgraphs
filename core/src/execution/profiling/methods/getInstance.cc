/*!
 * @file            getInstance.cc
 *
 * @brief           Implements TimerRegistry::getInstance() declared in
 *                  TimerRegistryClass.h.
 */

#ifdef VS_GRAPHS_BUILT_TIMERS

#include "execution/profiling/objects/TimerRegistryClass.h"

namespace vs_graphs::core::profiling
{

TimerRegistry &TimerRegistry::getInstance()
{
    /* Intentionally leaked: System::shutdown() does not join the worker
     * threads (a global bundle adjustment may still be running), so a timer
     * can still record a call after main() returns. A destroyed registry
     * would then be used after destruction. C++11 guarantees thread-safe
     * initialisation of a function-local static on first use. */
    static TimerRegistry &processTimerRegistry = *new TimerRegistry;
    return processTimerRegistry;
}

} /* namespace vs_graphs::core::profiling */

#endif /* VS_GRAPHS_BUILT_TIMERS */
