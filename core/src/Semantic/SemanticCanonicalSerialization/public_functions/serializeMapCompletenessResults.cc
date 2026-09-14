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
 * @file            serializeMapCompletenessResults.cc
 *
 * @brief           Implements serializeMapCompletenessResults(), declared in
 *                  public_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/public_functions.h"

#include <algorithm>
#include <utility>

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace ORB_SLAM3
{
namespace semantic
{

nlohmann::json serializeMapCompletenessResults(
    const std::vector<MapCompletenessResult> &results_in)
{
    nlohmann::json json;
    json["schema"] = SEMANTIC_COMPLETENESS_SCHEMA_VERSION;

    /* isMapCompletenessResultLessTotalOrder() compares every field this
     * module emits, not only mapId, so two results sharing mapId (a
     * genuine collision) still serialize in one fixed order. */
    std::vector<MapCompletenessResult> results = results_in;
    std::sort(results.begin(),
              results.end(),
              &isMapCompletenessResultLessTotalOrder);
    nlohmann::json resultsJson = nlohmann::json::array();
    for (const MapCompletenessResult &result : results)
    {
        resultsJson.push_back(serializeMapCompletenessResult(result));
    }
    json["results"] = std::move(resultsJson);

    return json;
}

} // namespace semantic
} // namespace ORB_SLAM3
