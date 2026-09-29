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
 * @file            evaluateMapCompleteness.cc
 *
 * @brief           Implements evaluateMapCompleteness(), declared in
 *                  public_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/public_functions.h"

#include <rclcpp/logging.hpp>
#include <utility>

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus evaluateMapCompleteness(
    const SemanticGraphSnapshot        &snapshot_in,
    std::vector<MapCompletenessResult> &completenessResults_out)
{
    std::vector<MapCompletenessResult> results;
    results.reserve(snapshot_in.maps.size());

    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        MapCompletenessResult result{};
        if (computeConservativeMapCompleteness(snapshot_in,
                                               mapSnapshot,
                                               result) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            // computeConservativeMapCompleteness cannot fail; continue as
            // before.
        }
        LegacyMapCompletenessResult legacyMapCompleteness{};
        if (computeLegacyMapCompleteness(snapshot_in,
                                         mapSnapshot,
                                         legacyMapCompleteness) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: computeLegacyMapCompleteness returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        result.legacy = legacyMapCompleteness;
        result.doLegacyAndConservativeDiverge =
            (result.legacy.isMapFullyModeled != result.isComplete);
        results.push_back(std::move(result));
    }

    completenessResults_out = results;
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
