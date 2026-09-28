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
double calcAverage(std::vector<double> times_in);

/*!
 * @brief        Standard deviation of per-frame millisecond timings.
 */
double calcDeviation(std::vector<double> times_in, double average_in);

/*!
 * @brief        Averages integer counters, skipping zero entries.
 */
double calcAverage(std::vector<int> values_in);

/*!
 * @brief        Standard deviation of integer counters, skipping zeros.
 */
double calcDeviation(std::vector<int> values_in, double average_in);
#endif

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_TRACKING_PRIVATE_FUNCTIONS_H */
