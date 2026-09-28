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
 * @file            serializeMapCompletenessResult.cc
 *
 * @brief           Implements serializeMapCompletenessResult(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

#include <algorithm>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

nlohmann::json
    serializeMapCompletenessResult(const MapCompletenessResult &value_in)
{
    nlohmann::json json;
    json["mapId"] = value_in.mapId;
    json["conservativeResult"] =
        static_cast<unsigned int>(value_in.conservativeResult);
    json["conservativeResultName"] =
        axiomResultName(value_in.conservativeResult);
    json["isComplete"] = value_in.isComplete;

    std::vector<ReasonCode> reasons = value_in.reasons;
    std::sort(reasons.begin(),
              reasons.end(),
              [](ReasonCode lhs_in, ReasonCode rhs_in)
              {
                  return static_cast<unsigned int>(lhs_in) <
                         static_cast<unsigned int>(rhs_in);
              });
    nlohmann::json reasonsJson     = nlohmann::json::array();
    nlohmann::json reasonNamesJson = nlohmann::json::array();
    for (const ReasonCode reason : reasons)
    {
        reasonsJson.push_back(static_cast<unsigned int>(reason));
        reasonNamesJson.push_back(reasonCodeName(reason));
    }
    json["reasons"]     = std::move(reasonsJson);
    json["reasonNames"] = std::move(reasonNamesJson);

    std::vector<EntityKey> relevantEntityKeys = value_in.relevantEntityKeys;
    std::sort(relevantEntityKeys.begin(), relevantEntityKeys.end());
    nlohmann::json relevantEntityKeysJson = nlohmann::json::array();
    for (const EntityKey &key : relevantEntityKeys)
    {
        relevantEntityKeysJson.push_back(serializeEntityKey(key));
    }
    json["relevantEntityKeys"] = std::move(relevantEntityKeysJson);

    json["confirmedRoomCount"]     = value_in.confirmedRoomCount;
    json["completeRoomCount"]      = value_in.completeRoomCount;
    json["prospectiveRoomCount"]   = value_in.prospectiveRoomCount;
    json["livePassageCount"]       = value_in.livePassageCount;
    json["fullyValidPassageCount"] = value_in.fullyValidPassageCount;
    json["legacy"] = serializeLegacyMapCompletenessResult(value_in.legacy);
    json["legacyAndConservativeDiverge"] =
        value_in.doLegacyAndConservativeDiverge;

    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
