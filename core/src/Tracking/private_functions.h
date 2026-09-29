/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  Tracking translation units.
 *
 * @note            These helpers were file-scope free functions inside the
 *                  REGISTER_TIMES region of Tracking.cc; external linkage
 *                  here is module-internal only. Names are kept verbatim
 *                  (identifier renaming is a separate step).
 */

#ifndef VS_GRAPHS_CORE_TRACKING_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_TRACKING_PRIVATE_FUNCTIONS_H

#include "TrackingStatus.h"
#include <vector>

namespace vs_graphs
{
namespace core
{

#ifdef REGISTER_TIMES
/*!
 * @brief        Averages per-frame millisecond timings.
 */
[[nodiscard]] TrackingStatus calcAverage(std::vector<double> times_in,
                                         double             &average_out);

/*!
 * @brief        Standard deviation of per-frame millisecond timings.
 */
[[nodiscard]] TrackingStatus calcDeviation(std::vector<double> times_in,
                                           double              average_in,
                                           double             &deviation_out);

/*!
 * @brief        Averages integer counters, skipping zero entries.
 */
[[nodiscard]] TrackingStatus calcAverage(std::vector<int> values_in,
                                         double          &average_out);

/*!
 * @brief        Standard deviation of integer counters, skipping zeros.
 */
[[nodiscard]] TrackingStatus calcDeviation(std::vector<int> values_in,
                                           double           average_in,
                                           double          &deviation_out);
#endif

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_TRACKING_PRIVATE_FUNCTIONS_H */
