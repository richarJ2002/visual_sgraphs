/*!
 * @file            recordCall.cc
 *
 * @brief           Implements TimerRegistry::recordCall() declared in
 *                  TimerRegistryClass.h.
 */

#ifdef VS_GRAPHS_BUILT_TIMERS

#include "execution/profiling/objects/TimerRegistryClass.h"

#include <algorithm>
#include <mutex>
#include <string>

namespace vs_graphs::core::profiling
{

void TimerRegistry::recordCall(const char  *p_sourceFile_in,
                               const char  *p_functionName_in,
                               const double callDuration_ms_in) noexcept
{
    try
    {
        const std::lock_guard<std::mutex> registryLock(registryMutex);
        TimerStatistics                  &functionStatistics =
            timerStatistics[{std::string(p_sourceFile_in),
                             std::string(p_functionName_in)}];
        functionStatistics.callCount++;
        functionStatistics.totalDuration_ms += callDuration_ms_in;
        functionStatistics.maximumDuration_ms =
            std::max(functionStatistics.maximumDuration_ms, callDuration_ms_in);
    }
    catch (...)
    {
        /* Timing is diagnostic only: drop the sample rather than change the
         * behaviour of the timed code. */
    }
}

} /* namespace vs_graphs::core::profiling */

#endif /* VS_GRAPHS_BUILT_TIMERS */
