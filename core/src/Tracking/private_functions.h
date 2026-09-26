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

#include <vector>

namespace vs_graphs
{
namespace core
{

#ifdef REGISTER_TIMES
/*!
 * @brief        Averages per-frame millisecond timings.
 */
double calcAverage(std::vector<double> v_times);

/*!
 * @brief        Standard deviation of per-frame millisecond timings.
 */
double calcDeviation(std::vector<double> v_times, double average);

/*!
 * @brief        Averages integer counters, skipping zero entries.
 */
double calcAverage(std::vector<int> v_values);

/*!
 * @brief        Standard deviation of integer counters, skipping zeros.
 */
double calcDeviation(std::vector<int> v_values, double average);
#endif

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_TRACKING_PRIVATE_FUNCTIONS_H */
