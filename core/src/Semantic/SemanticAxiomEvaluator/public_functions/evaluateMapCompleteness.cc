/**
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

#include <utility>

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace ORB_SLAM3
{
namespace semantic
{

std::vector<MapCompletenessResult>
    evaluateMapCompleteness(const SemanticGraphSnapshot &snapshot_in)
{
    std::vector<MapCompletenessResult> results;
    results.reserve(snapshot_in.maps.size());

    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        MapCompletenessResult result =
            computeConservativeMapCompleteness(snapshot_in, mapSnapshot);
        result.legacy = computeLegacyMapCompleteness(snapshot_in, mapSnapshot);
        result.legacyAndConservativeDiverge =
            (result.legacy.mapFullyModeled != result.isComplete);
        results.push_back(std::move(result));
    }

    return results;
}

} // namespace semantic
} // namespace ORB_SLAM3
