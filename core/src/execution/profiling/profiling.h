/*!
 * @file            profiling.h
 *
 * @brief           Public interface of the profiling module: the built-in
 *                  function timers of an R&D timers build.
 *
 *                  Both macros expand to nothing unless the build defines
 *                  VS_GRAPHS_BUILT_TIMERS, so a normal build carries no trace
 *                  of the timers (C++ standard section 11.3).
 */

#ifndef VS_GRAPHS_CORE_SRC_PROFILING_PROFILING_H
#define VS_GRAPHS_CORE_SRC_PROFILING_PROFILING_H

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

/*!
 * @brief           Writes the timer report of this process into the test-run
 *                  folder `directory`; does nothing when it is empty.
 */
#define PROFILE_WRITE_REPORT(directory)                                        \
    vs_graphs::core::profiling::TimerRegistry::getInstance().writeReport(      \
        directory)

#else

#define PROFILE_FUNCTION()
#define PROFILE_WRITE_REPORT(directory)

#endif /* VS_GRAPHS_BUILT_TIMERS */

#endif /* VS_GRAPHS_CORE_SRC_PROFILING_PROFILING_H */
