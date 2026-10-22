/*!
 * @file            public_functions.h
 *
 * @brief           Declares RGB-D observability summary formatting.
 *
 * @date            09/10/2026
 */

#ifndef VS_GRAPHS_OBSERVABILITY_PUBLIC_FUNCTIONS_H
#define VS_GRAPHS_OBSERVABILITY_PUBLIC_FUNCTIONS_H

/* Function Includes */
/* None */

/* Object Include */
#include "observability/objects/ObservabilitySnapshotStruct.h"

/* Data include */
#include <string>

/* Generic Libraries */
/* None */

namespace vs_graphs::observability
{

/*!
 * @brief           Formats one stable, single-line, key-value summary.
 *
 * @param[in]       snapshot_in
 *                  Copy of cumulative callback and worker counters.
 *
 * @param[in]       event_in
 *                  Stable event label included in the summary.
 *
 * @return          One key-value summary line for diagnostics.
 */
[[nodiscard]] std::string
    formatRgbdObservabilitySummary(const RgbdObservabilitySnapshot &snapshot_in,
                                   const std::string               &event_in);

} /* namespace vs_graphs::observability */

#endif /* VS_GRAPHS_OBSERVABILITY_PUBLIC_FUNCTIONS_H */
