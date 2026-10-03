/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            SemanticDiagnosticState.h
 *
 * @brief           Declares the caller-owned, opaque state
 *                  buildSemanticDiagnosticUpdate() reads and updates across
 *                  calls to detect finding/topology transitions and pace the
 *                  heartbeat.
 */

#ifndef SEMANTIC_DIAGNOSTICS_STATE_H
#define SEMANTIC_DIAGNOSTICS_STATE_H

#include <cstdint>
#include <optional>
#include <string>

#include "Semantic/SemanticAxiomEvaluator/objects/AxiomEvaluationReport.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Retained across calls to buildSemanticDiagnosticUpdate() so
 *                  it can detect appeared/changed/resolved FAIL findings and a
 *                  topology digest change relative to the last cycle it
 *                  actually emitted, and pace the heartbeat. A
 *                  default-constructed instance represents "no cycle logged
 *                  yet."
 */
struct SemanticDiagnosticState
{
  public:
    /*!
     * @brief           Complete evaluation report as of the last emitted
     *                  summary; absent before the first call.
     */
    std::optional<AxiomEvaluationReport> lastLoggedEvaluationReport;

    /*!
     * @brief           Canonical topology digest as of the last emitted
     *                  summary.
     */
    std::optional<std::string> lastLoggedTopologyDigest;

    /*!
     * @brief           Canonical full-geometry digest as of the last emitted
     *                  summary.
     */
    std::optional<std::string> lastLoggedFullGeometryDigest;

    /*!
     * @brief           Number of calls, including the current one, since (and
     *                  including) the last emitted summary or heartbeat; a
     *                  heartbeat is emitted when this reaches
     *                  kDiagnosticHeartbeatCycles (see public_functions.h).
     *                  Reset to 1 by every call that emits.
     */
    std::uint64_t cyclesSinceLastDiagnosticSummary{0U};
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_DIAGNOSTICS_STATE_H
