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
 * @file            entityKeysToJson.cc
 *
 * @brief           Implements entityKeysToJson(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticDiagnostics/private_functions.h"

#include <algorithm>

#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticDiagnosticsStatus
    entityKeysToJson(const std::vector<EntityKey> &keys_in,
                     nlohmann::json               &json_out)
{
    std::vector<EntityKey> sortedKeys = keys_in;
    std::sort(sortedKeys.begin(), sortedKeys.end());

    nlohmann::json json = nlohmann::json::array();
    for (const EntityKey &key : sortedKeys)
    {
        std::string entityKindName2{};
        if (entityKindName(key.kind, entityKindName2) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            // entityKindName cannot fail; continue as before.
        }
        json.push_back({{"kind", static_cast<unsigned int>(key.kind)},
                        {"kindName", entityKindName2},
                        {"mapId", key.mapId},
                        {"entityId", key.entityId}});
    }
    json_out = json;
    return SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
