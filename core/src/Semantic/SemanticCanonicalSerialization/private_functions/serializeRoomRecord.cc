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
 * @file            serializeRoomRecord.cc
 *
 * @brief           Implements serializeRoomRecord(), declared in
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

nlohmann::json serializeRoomRecord(const RoomRecord &value_in,
                                   bool              includeGeometry_in)
{
    nlohmann::json json;
    json["key"]                 = serializeEntityKey(value_in.key);
    json["isLive"]              = value_in.isLive;
    json["isDetectedMember"]    = value_in.isDetectedMember;
    json["isMarkerBasedMember"] = value_in.isMarkerBasedMember;
    if (value_in.declaredMapId.has_value())
    {
        json["declaredMapId"] = *value_in.declaredMapId;
    }
    json["variant"]        = static_cast<int>(value_in.variant);
    json["boundaryStatus"] = static_cast<unsigned int>(value_in.boundaryStatus);

    std::vector<RawPlaneRef> wallRefs = value_in.wallRefs;
    std::sort(wallRefs.begin(), wallRefs.end(), &isRawPlaneRefLess);
    nlohmann::json wallRefsJson = nlohmann::json::array();
    for (const RawPlaneRef &wallRef : wallRefs)
    {
        wallRefsJson.push_back(serializeRawPlaneRef(wallRef));
    }
    json["wallRefs"] = std::move(wallRefsJson);

    std::vector<EntityRef> passageRefs = value_in.passageRefs;
    std::sort(passageRefs.begin(), passageRefs.end(), &isEntityRefLess);
    nlohmann::json passageRefsJson = nlohmann::json::array();
    for (const EntityRef &passageRef : passageRefs)
    {
        passageRefsJson.push_back(serializeEntityRef(passageRef));
    }
    json["passageRefs"] = std::move(passageRefsJson);

    json["floorRef"]       = serializeEntityRef(value_in.floorRef);
    json["groundPlaneRef"] = serializeRawPlaneRef(value_in.groundPlaneRef);
    json["creationProvenanceReason"] =
        static_cast<unsigned int>(value_in.creationProvenanceReason);
    json["creationProvenanceReasonName"] =
        unavailableReasonName(value_in.creationProvenanceReason);

    if (includeGeometry_in)
    {
        json["centroid_World_m"] = serializeVector3d(value_in.centroid_World_m);

        nlohmann::json boundaryCornersJson = nlohmann::json::array();
        for (const Eigen::Vector3d &corner : value_in.boundaryCorners_World_m)
        {
            boundaryCornersJson.push_back(serializeVector3d(corner));
        }
        json["boundaryCorners_World_m"] = std::move(boundaryCornersJson);

        nlohmann::json observationGapsJson = nlohmann::json::array();
        for (const Room::ObservationGap &gap : value_in.observationGaps)
        {
            nlohmann::json gapJson;
            gapJson["startAngle_rad"] = serializeDouble(gap.startAngle_rad);
            gapJson["spanAngle_rad"]  = serializeDouble(gap.spanAngle_rad);
            observationGapsJson.push_back(std::move(gapJson));
        }
        json["observationGaps"] = std::move(observationGapsJson);
    }

    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
