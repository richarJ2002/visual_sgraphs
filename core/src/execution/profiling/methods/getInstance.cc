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
    /*!
     * Intentionally leaked. On a normal exit ~System joins every worker
     * thread before main() returns, but the fatal exit() calls (for example
     * trackRGBD() with the wrong sensor type) end the process while the
     * worker threads still run and record calls. A registry destroyed by
     * exit() would then be used after destruction. C++11 guarantees
     * thread-safe initialisation of a function-local static on first use.
     */
    static TimerRegistry &processTimerRegistry = *new TimerRegistry;
    return processTimerRegistry;
}

} /* namespace vs_graphs::core::profiling */

#endif /* VS_GRAPHS_BUILT_TIMERS */
