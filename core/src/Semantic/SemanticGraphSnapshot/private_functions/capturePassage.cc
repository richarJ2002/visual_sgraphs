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
 * @file            capturePassage.cc
 *
 * @brief           Implements capturePassage(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

#include "Map.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticGraphSnapshotStatus capturePassage(Passage          *p_passage_in,
                                           long unsigned int mapId_in,
                                           PassageRecord    &passageRecord_out)
{
    PassageRecord record;
    int           passage_inId{};
    if (p_passage_in->getId(passage_inId) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    EntityKey key2{};
    if (makeKey(EntityKind::PASSAGE, mapId_in, passage_inId, key2) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeKey returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    record.key = key2;
    bool passage_inIsBad{};
    if (p_passage_in->isBad(passage_inIsBad) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    record.isLive = !passage_inIsBad;

    core::Map *p_declaredMap = nullptr;
    if (p_passage_in->getMap(p_declaredMap) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_declaredMap != nullptr)
    {
        unsigned long declaredMapId2{};
        if (p_declaredMap->getId(declaredMapId2) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        record.declaredMapId = declaredMapId2;
    }

    Passage::PassageVariant passage_inPassageType{};
    if (p_passage_in->getPassageType(passage_inPassageType) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPassageType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    record.passageType = passage_inPassageType;
    g2o::Plane3D passage_inGlobalEquation{};
    if (p_passage_in->getGlobalEquation(passage_inGlobalEquation) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    record.equation_World = passage_inGlobalEquation.coeffs();
    Eigen::Vector3d passage_inCentroid{};
    if (p_passage_in->getCentroid(passage_inCentroid) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    record.centroid_World_m = passage_inCentroid;
    double passage_inWidth{};
    if (p_passage_in->getWidth(passage_inWidth) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWidth returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    record.width_m = passage_inWidth;
    double passage_inHeight{};
    if (p_passage_in->getHeight(passage_inHeight) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHeight returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    record.height_m = passage_inHeight;
    bool passage_inIsPassable{};
    if (p_passage_in->isPassable(passage_inIsPassable) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isPassable returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    record.isPassable = passage_inIsPassable;

    std::vector<vs_graphs::core::geometric::Plane *> passage_inAssociateWalls{};
    if (p_passage_in->getAssociateWalls(passage_inAssociateWalls) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAssociateWalls returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_wall : passage_inAssociateWalls)
    {
        if (appendWallRef(p_wall, record.associateWallRefs) !=
            SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: appendWallRef returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }
    std::sort(record.associateWallRefs.begin(),
              record.associateWallRefs.end(),
              &isRawPlaneRefLess);

    vs_graphs::core::geometric::Plane *p_passage_inAssociateDoor = nullptr;
    if (p_passage_in->getAssociateDoor(p_passage_inAssociateDoor) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAssociateDoor returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    RawPlaneRef rawPlaneRef2{};
    if (rawPlaneRef(p_passage_inAssociateDoor, rawPlaneRef2) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: rawPlaneRef returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    record.associateDoorRef = rawPlaneRef2;

    Passage::KnownSideProvenance provenance{};
    if (p_passage_in->getKnownSideProvenance(provenance) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKnownSideProvenance returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    EntityRef entityRef{};
    if (entityRefForRoom(provenance.p_room, entityRef) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: entityRefForRoom returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    record.knownSideRoomRef = entityRef;
    bool provenanceHasDirection{};
    if (provenance.hasDirection(provenanceHasDirection) !=
        KnownSideProvenanceStatus::KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (provenanceHasDirection)
    {
        record.knownSideDirection_World = provenance.direction_World;
    }

    vs_graphs::core::semantic::Room *p_passage_inProspectiveRoom = nullptr;
    if (p_passage_in->getProspectiveRoom(p_passage_inProspectiveRoom) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getProspectiveRoom returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    EntityRef entityRef2{};
    if (entityRefForRoom(p_passage_inProspectiveRoom, entityRef2) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: entityRefForRoom returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    record.prospectiveRoomRef = entityRef2;

    std::size_t passage_inTraversalKnownToFarCount{};
    if (p_passage_in->getTraversalKnownToFarCount(
            passage_inTraversalKnownToFarCount) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTraversalKnownToFarCount returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    record.traversalKnownToFarCount = passage_inTraversalKnownToFarCount;
    std::size_t passage_inTraversalFarToKnownCount{};
    if (p_passage_in->getTraversalFarToKnownCount(
            passage_inTraversalFarToKnownCount) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTraversalFarToKnownCount returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    record.traversalFarToKnownCount = passage_inTraversalFarToKnownCount;
    std::size_t passage_inTraversalUnknownCount{};
    if (p_passage_in->getTraversalUnknownCount(
            passage_inTraversalUnknownCount) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTraversalUnknownCount returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    record.traversalUnknownCount = passage_inTraversalUnknownCount;

    passageRecord_out = record;
    return SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
