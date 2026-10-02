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
 * @file            serializeFloorRecord.cc
 *
 * @brief           Implements serializeFloorRecord(), declared in
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

nlohmann::json serializeFloorRecord(const FloorRecord &value_in,
                                    bool               includeGeometry_in)
{
    nlohmann::json json;
    json["key"] = serializeEntityKey(value_in.key);
    if (value_in.declaredMapId.has_value())
    {
        json["declaredMapId"] = *value_in.declaredMapId;
    }

    std::vector<EntityRef> roomRefs = value_in.roomRefs;
    std::sort(roomRefs.begin(), roomRefs.end(), &isEntityRefLess);
    nlohmann::json roomRefsJson = nlohmann::json::array();
    for (const EntityRef &roomReference : roomRefs)
    {
        roomRefsJson.push_back(serializeEntityRef(roomReference));
    }
    json["roomRefs"] = std::move(roomRefsJson);

    if (includeGeometry_in)
    {
        json["centroid_World_m"] =
            serializeVector3d(value_in.floorCentroid_world_m);
        if (value_in.planeIdentity.has_value())
        {
            nlohmann::json planeIdentityJson;
            planeIdentityJson["equation_World"] =
                serializeVector4d(value_in.planeIdentity->planeEquation_world);
            planeIdentityJson["finiteSupportCount"] =
                value_in.planeIdentity->finiteSupportCount;
            planeIdentityJson["observationCount"] =
                value_in.planeIdentity->observationCount;
            json["planeIdentity"] = std::move(planeIdentityJson);
        }
    }

    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
