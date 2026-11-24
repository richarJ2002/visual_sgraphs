/*!
 * @file            ScopedTimerClass.h
 *
 * @brief           Declares the timer that measures one function call.
 */

#ifndef VS_GRAPHS_CORE_SRC_PROFILING_OBJECTS_SCOPED_TIMER_CLASS_H
#define VS_GRAPHS_CORE_SRC_PROFILING_OBJECTS_SCOPED_TIMER_CLASS_H

#include "execution/profiling/objects/TimerRegistryClass.h"

#include <chrono>

namespace vs_graphs::core::profiling
{
/*!
 * @brief           Measures the time from its construction to its destruction
 *                  and records it in the TimerRegistry.
 *
 *                  It uses the steady (wall) clock, not the ROS sim clock: the
 *                  timers measure compute cost, which is real time spent on
 *                  the CPU or waiting, whatever the simulation speed.
 */
class ScopedTimer
{
  public:
    /*!
     * @brief           Starts timing one call.
     *
     * @param[in]       p_sourceFile_in
     *                  Source file of the timed function (__FILE__); a string
     *                  literal that outlives the timer.
     *
     * @param[in]       p_functionName_in
     *                  Name of the timed function (__func__); outlives the
     *                  timer.
     */
    ScopedTimer(const char *p_sourceFile_in,
                const char *p_functionName_in) noexcept :
        p_sourceFile(p_sourceFile_in),
        p_functionName(p_functionName_in),
        startTime(std::chrono::steady_clock::now())
    {}

    /*!
     * @brief           Records the call duration in the registry.
     */
    ~ScopedTimer() noexcept
    {
        const std::chrono::duration<double, std::milli> callDuration_ms =
            std::chrono::steady_clock::now() - startTime;
        TimerRegistry::getInstance().recordCall(p_sourceFile,
                                                p_functionName,
                                                callDuration_ms.count());
    }

    ScopedTimer(const ScopedTimer &otherScopedTimer_in)            = delete;
    ScopedTimer &operator=(const ScopedTimer &otherScopedTimer_in) = delete;

  private:
    /*! @brief           Source file of the timed function; not owned. */
    const char *p_sourceFile;

    /*! @brief           Name of the timed function; not owned. */
    const char *p_functionName;

    /*! @brief           Steady-clock time the call started. */
    std::chrono::steady_clock::time_point startTime;
};
} /* namespace vs_graphs::core::profiling */

#endif /* VS_GRAPHS_CORE_SRC_PROFILING_OBJECTS_SCOPED_TIMER_CLASS_H */
