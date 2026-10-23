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
    /* C++11 guarantees thread-safe initialisation of a function-local static
     * on first use. */
    static TimerRegistry processTimerRegistry;
    return processTimerRegistry;
}

} /* namespace vs_graphs::core::profiling */

#endif /* VS_GRAPHS_BUILT_TIMERS */
