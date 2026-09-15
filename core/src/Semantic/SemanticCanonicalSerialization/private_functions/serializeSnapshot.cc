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
 * @file            serializeSnapshot.cc
 *
 * @brief           Implements serializeSnapshot(), declared in
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

nlohmann::json serializeSnapshot(const SemanticGraphSnapshot &snapshot_in,
                                 bool includeGeometry_in)
{
    nlohmann::json json;
    if (snapshot_in.currentMapId.has_value())
    {
        json["currentMapId"] = *snapshot_in.currentMapId;
    }
    json["currentMapStatus"] =
        static_cast<unsigned int>(snapshot_in.currentMapStatus);

    /* isMapSnapshotLessTotalOrder() compares every field the requested
     * projection emits, not only mapId: even though Map::nNextId being a
     * monotonically increasing static counter makes a true mapId collision
     * within one live Atlas capture very unlikely in practice, this
     * serializer does not rely on that source-model invariant to guarantee
     * determinism -- a defensive full-value tiebreak is cheap and keeps the
     * output byte-identical under input permutation regardless. */
    std::vector<MapSnapshot> maps = snapshot_in.maps;
    std::sort(maps.begin(),
              maps.end(),
              [includeGeometry_in](const MapSnapshot &lhs_in,
                                   const MapSnapshot &rhs_in) {
                  return isMapSnapshotLessTotalOrder(lhs_in,
                                                     rhs_in,
                                                     includeGeometry_in);
              });
    nlohmann::json mapsJson = nlohmann::json::array();
    for (const MapSnapshot &map : maps)
    {
        mapsJson.push_back(serializeMapSnapshot(map, includeGeometry_in));
    }
    json["maps"] = std::move(mapsJson);

    json["badRetiredMapVisibility"] =
        static_cast<unsigned int>(snapshot_in.badRetiredMapVisibility);
    json["roomContextHistoryReason"] =
        static_cast<unsigned int>(snapshot_in.roomContextHistoryReason);

    std::vector<OpenPassageHypothesisRecord> openPassageHypotheses =
        snapshot_in.managerPrivateOpenPassageHypotheses;
    std::sort(openPassageHypotheses.begin(),
              openPassageHypotheses.end(),
              includeGeometry_in
                  ? &isOpenPassageHypothesisRecordLessFullGeometry
                  : &isOpenPassageHypothesisRecordLessTopologyOnly);
    nlohmann::json openPassageHypothesesJson = nlohmann::json::array();
    for (const OpenPassageHypothesisRecord &hypothesis : openPassageHypotheses)
    {
        openPassageHypothesesJson.push_back(
            serializeOpenPassageHypothesisRecord(hypothesis,
                                                 includeGeometry_in));
    }
    json["managerPrivateOpenPassageHypotheses"] =
        std::move(openPassageHypothesesJson);
    json["managerPrivateOpenPassageHypothesesReason"] =
        static_cast<unsigned int>(
            snapshot_in.managerPrivateOpenPassageHypothesesReason);

    std::vector<UnresolvedWallHypothesisRecord> unresolvedWallHypotheses =
        snapshot_in.managerPrivateUnresolvedWallHypotheses;
    std::sort(unresolvedWallHypotheses.begin(),
              unresolvedWallHypotheses.end(),
              &isUnresolvedWallHypothesisRecordLessTotalOrder);
    nlohmann::json unresolvedWallHypothesesJson = nlohmann::json::array();
    for (const UnresolvedWallHypothesisRecord &hypothesis :
         unresolvedWallHypotheses)
    {
        unresolvedWallHypothesesJson.push_back(
            serializeUnresolvedWallHypothesisRecord(hypothesis));
    }
    json["managerPrivateUnresolvedWallHypotheses"] =
        std::move(unresolvedWallHypothesesJson);
    json["managerPrivateUnresolvedWallHypothesesReason"] =
        static_cast<unsigned int>(
            snapshot_in.managerPrivateUnresolvedWallHypothesesReason);

    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
