/*!
 * @file            TimerRegistryClass.h
 *
 * @brief           Declares the process-wide store of function timings.
 */

#ifndef VS_GRAPHS_CORE_SRC_PROFILING_OBJECTS_TIMER_REGISTRY_CLASS_H
#define VS_GRAPHS_CORE_SRC_PROFILING_OBJECTS_TIMER_REGISTRY_CLASS_H

#include "execution/profiling/objects/TimerStatisticsStruct.h"

#include <map>
#include <mutex>
#include <string>
#include <utility>

namespace vs_graphs::core::profiling
{
/*!
 * @brief           Accumulates the timings of every tagged function of the
 *                  process and writes them as one CSV report. Thread-safe.
 */
class TimerRegistry
{
  public:
    TimerRegistry(const TimerRegistry &otherTimerRegistry_in) = delete;
    TimerRegistry &
        operator=(const TimerRegistry &otherTimerRegistry_in) = delete;

    /*!
     * @brief           Returns the single registry of the process, created on
     *                  first use and destroyed at process exit.
     *
     * @return          The process-wide registry.
     */
    static TimerRegistry &getInstance();

    /*!
     * @brief           Adds one completed call to the statistics of a
     *                  function. A sample that cannot be stored (out of
     *                  memory) is dropped, so timing never changes the
     *                  behaviour of the timed code.
     *
     * @param[in]       p_sourceFile_in
     *                  Source file of the function (__FILE__); not null.
     *
     * @param[in]       p_functionName_in
     *                  Name of the function (__func__); not null.
     *
     * @param[in]       callDuration_ms_in
     *                  Steady-clock duration of the call, milliseconds.
     */
    void recordCall(const char *p_sourceFile_in,
                    const char *p_functionName_in,
                    double      callDuration_ms_in) noexcept;

    /*!
     * @brief           Writes
     *                  `<testRunDirectory_in>/archive/profiling/timers/
     *                  vs_graphs.csv` with the header
     *                  `function,file,calls,total_ms,mean_ms,max_ms`, one row
     *                  per function, creating the folders. Each file is
     *                  relative to the package source root. Logs and returns
     *                  on a write failure.
     *
     * @param[in]       testRunDirectory_in
     *                  Test-run folder; empty writes nothing.
     */
    void writeReport(const std::string &testRunDirectory_in);

  private:
    /*!
     * @brief           Creates an empty registry; only getInstance() does.
     */
    TimerRegistry() = default;

    /*!
     * @brief           Guards timerStatistics.
     *
     * ponytail: one global lock for every timed call; per-thread maps merged
     * at report time if lock contention ever shows in the timings.
     */
    std::mutex registryMutex;

    /*!
     * @brief           Statistics keyed on (source file, function name);
     *                  __func__ alone is ambiguous (several run loops).
     */
    std::map<std::pair<std::string, std::string>, TimerStatistics>
        timerStatistics;
};
} /* namespace vs_graphs::core::profiling */

#endif /* VS_GRAPHS_CORE_SRC_PROFILING_OBJECTS_TIMER_REGISTRY_CLASS_H */
