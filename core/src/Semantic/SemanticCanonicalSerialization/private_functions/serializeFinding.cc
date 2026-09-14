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
 * @file            serializeFinding.cc
 *
 * @brief           Implements serializeFinding(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

#include <algorithm>
#include <utility>

namespace ORB_SLAM3
{
namespace semantic
{

nlohmann::json serializeFinding(const Finding &value_in)
{
    nlohmann::json json;
    json["id"]             = value_in.id;
    json["axiomCode"]      = static_cast<unsigned int>(value_in.axiomCode);
    json["axiomCodeName"]  = axiomCodeName(value_in.axiomCode);
    json["result"]         = static_cast<unsigned int>(value_in.result);
    json["resultName"]     = axiomResultName(value_in.result);
    json["classification"] = static_cast<unsigned int>(value_in.classification);
    json["classificationName"] = axiomClassName(value_in.classification);
    json["reasonCode"]         = static_cast<unsigned int>(value_in.reasonCode);
    json["reasonCodeName"]     = reasonCodeName(value_in.reasonCode);

    std::vector<EntityKey> involvedKeys = value_in.involvedKeys;
    std::sort(involvedKeys.begin(), involvedKeys.end());
    nlohmann::json involvedKeysJson = nlohmann::json::array();
    for (const EntityKey &key : involvedKeys)
    {
        involvedKeysJson.push_back(serializeEntityKey(key));
    }
    json["involvedKeys"] = std::move(involvedKeysJson);

    nlohmann::json evidenceJson;
    if (value_in.evidence.observedCount.has_value())
    {
        evidenceJson["observedCount"] = *value_in.evidence.observedCount;
    }
    if (value_in.evidence.expectedCount.has_value())
    {
        evidenceJson["expectedCount"] = *value_in.evidence.expectedCount;
    }
    if (value_in.evidence.numericValue.has_value())
    {
        evidenceJson["numericValue"] =
            serializeDouble(*value_in.evidence.numericValue);
    }
    json["evidence"] = std::move(evidenceJson);

    return json;
}

} // namespace semantic
} // namespace ORB_SLAM3
