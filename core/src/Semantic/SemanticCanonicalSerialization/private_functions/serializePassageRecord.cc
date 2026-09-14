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
 * @file            serializePassageRecord.cc
 *
 * @brief           Implements serializePassageRecord(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

#include <algorithm>
#include <utility>

namespace ORB_SLAM3
{
namespace semantic
{

nlohmann::json serializePassageRecord(const PassageRecord &value_in,
                                      bool                 includeGeometry_in)
{
    nlohmann::json json;
    json["key"]    = serializeEntityKey(value_in.key);
    json["isLive"] = value_in.isLive;
    if (value_in.declaredMapId.has_value())
    {
        json["declaredMapId"] = *value_in.declaredMapId;
    }
    json["passageType"] = static_cast<int>(value_in.passageType);
    json["passable"]    = value_in.passable;

    std::vector<RawPlaneRef> associateWallRefs = value_in.associateWallRefs;
    std::sort(associateWallRefs.begin(),
              associateWallRefs.end(),
              &isRawPlaneRefLess);
    nlohmann::json associateWallRefsJson = nlohmann::json::array();
    for (const RawPlaneRef &wallRef : associateWallRefs)
    {
        associateWallRefsJson.push_back(serializeRawPlaneRef(wallRef));
    }
    json["associateWallRefs"] = std::move(associateWallRefsJson);

    json["associateDoorRef"] = serializeRawPlaneRef(value_in.associateDoorRef);
    json["knownSideRoomRef"] = serializeEntityRef(value_in.knownSideRoomRef);
    json["prospectiveRoomRef"] =
        serializeEntityRef(value_in.prospectiveRoomRef);
    json["traversalKnownToFarCount"] = value_in.traversalKnownToFarCount;
    json["traversalFarToKnownCount"] = value_in.traversalFarToKnownCount;
    json["traversalUnknownCount"]    = value_in.traversalUnknownCount;
    json["endpointSlotReason"] =
        static_cast<unsigned int>(value_in.endpointSlotReason);
    json["endpointSlotReasonName"] =
        unavailableReasonName(value_in.endpointSlotReason);

    if (includeGeometry_in)
    {
        json["equation_World"]   = serializeVector4d(value_in.equation_World);
        json["centroid_World_m"] = serializeVector3d(value_in.centroid_World_m);
        json["width_m"]          = serializeDouble(value_in.width_m);
        json["height_m"]         = serializeDouble(value_in.height_m);
        if (value_in.knownSideDirection_World.has_value())
        {
            json["knownSideDirection_World"] =
                serializeVector3d(*value_in.knownSideDirection_World);
        }
    }

    return json;
}

} // namespace semantic
} // namespace ORB_SLAM3
