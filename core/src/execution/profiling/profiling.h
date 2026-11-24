/*!
 * @file            profiling.h
 *
 * @brief           Public interface of the profiling module: the built-in
 *                  function timers of an R&D timers build.
 *
 *                  PROFILE_FUNCTION() expands to nothing and
 *                  writeTimerReport() does nothing unless the build defines
 *                  VS_GRAPHS_BUILT_TIMERS, so a normal build carries no trace
 *                  of the timers (C++ standard section 11.3).
 */

#ifndef VS_GRAPHS_CORE_SRC_PROFILING_PROFILING_H
#define VS_GRAPHS_CORE_SRC_PROFILING_PROFILING_H

#include <string>

namespace vs_graphs::core::profiling
{
/*!
 * @brief           Writes
 *                  `<testRunDirectory_in>/archive/profiling/timers/
 *                  vs_graphs.csv` with the header
 *                  `function,file,calls,total_ms,mean_ms,max_ms`, one row per
 *                  timed function, creating the folders. Each file is
 *                  relative to the package source root. Logs and returns on a
 *                  write failure; does nothing in a build without timers.
 *
 * @param[in]       testRunDirectory_in
 *                  Test-run folder; empty writes nothing.
 */
void writeTimerReport(const std::string &testRunDirectory_in);
} /* namespace vs_graphs::core::profiling */

#ifdef VS_GRAPHS_BUILT_TIMERS

#include "execution/profiling/objects/ScopedTimerClass.h"
#include "execution/profiling/objects/TimerRegistryClass.h"

/*!
 * @brief           Times the rest of the enclosing function body; the first
 *                  statement of a tagged function.
 */
#define PROFILE_FUNCTION()                                                     \
    const vs_graphs::core::profiling::ScopedTimer profileScopedTimer(__FILE__, \
                                                                     __func__)

#else

#define PROFILE_FUNCTION()

#endif /* VS_GRAPHS_BUILT_TIMERS */

#endif /* VS_GRAPHS_CORE_SRC_PROFILING_PROFILING_H */
