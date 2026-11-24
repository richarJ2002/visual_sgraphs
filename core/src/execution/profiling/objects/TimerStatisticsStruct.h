/*!
 * @file            TimerStatisticsStruct.h
 *
 * @brief           Declares the accumulated timings of one tagged function.
 */

#ifndef VS_GRAPHS_CORE_SRC_PROFILING_OBJECTS_TIMER_STATISTICS_STRUCT_H
#define VS_GRAPHS_CORE_SRC_PROFILING_OBJECTS_TIMER_STATISTICS_STRUCT_H

#include <cstdint>

namespace vs_graphs::core::profiling
{
/*!
 * @brief           Accumulated steady-clock timings of one tagged function.
 */
struct TimerStatistics
{
  public:
    /*! @brief           Number of completed calls. */
    std::uint64_t callCount{0U};

    /*! @brief           Sum of all call durations, milliseconds. */
    double totalDuration_ms{0.0};

    /*! @brief           Longest single call duration, milliseconds. */
    double maximumDuration_ms{0.0};
};
} /* namespace vs_graphs::core::profiling */

#endif /* VS_GRAPHS_CORE_SRC_PROFILING_OBJECTS_TIMER_STATISTICS_STRUCT_H */
