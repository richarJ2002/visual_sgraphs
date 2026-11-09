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
 * @file            capturePassageEntry.cc
 *
 * @brief           Implements capturePassageEntry(), declared in
 *                  semantics_manager/graph/snapshots/private_functions/
 *                  private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/snapshots/private_functions/private_functions.h"

#include <optional>
#include <vector>

#include <rclcpp/logging.hpp>

#include "mapping/geometric/Plane/Plane.h"

namespace vs_graphs
{
namespace core
{

void capturePassageEntry(semantic::Passage      *p_passage_inout,
                         SceneGraphPassageEntry &passageEntry_out)
{
    passageEntry_out = SceneGraphPassageEntry{};
    std::vector<geometric::Plane *>        walls{};
    semantic::Passage::KnownSideProvenance knownSide{};
    if (p_passage_inout->getId(passageEntry_out.id) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->isBad(passageEntry_out.isBad) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->isPassable(passageEntry_out.isPassable) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->isRecoveryProxy(passageEntry_out.isRecoveryProxy) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->getTraversalEvidence(passageEntry_out.isTraversed) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->isBlocked(passageEntry_out.isBlocked) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->getWidth(passageEntry_out.width_m) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->getHeight(passageEntry_out.height_m) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->getCentroid(passageEntry_out.centroid_world_m) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->getProspectiveRoomId(
            passageEntry_out.prospectiveRoomId) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->getKnownSideProvenance(knownSide) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->getTraversalKnownToFarCount(
            passageEntry_out.traversalKnownToFarCount) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->getTraversalFarToKnownCount(
            passageEntry_out.traversalFarToKnownCount) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->getTraversalUnknownCount(
            passageEntry_out.traversalUnknownCount) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_inout->getAssociateWalls(walls) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: a passage getter returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    int knownSideRoomId{};
    if (knownSide.p_room != nullptr &&
        knownSide.p_room->getId(knownSideRoomId) ==
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        passageEntry_out.knownSideRoomId = knownSideRoomId;
    }
    for (geometric::Plane *p_wall : walls)
    {
        int wallId{};
        if (p_wall != nullptr &&
            p_wall->getId(wallId) ==
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            passageEntry_out.associatedWallIds.push_back(wallId);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
