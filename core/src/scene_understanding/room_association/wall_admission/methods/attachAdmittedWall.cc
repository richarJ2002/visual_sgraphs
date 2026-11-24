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
 * @file            attachAdmittedWall.cc
 *
 * @brief           Implements SemanticsManager::attachAdmittedWall(), declared
 *                  in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "configuration/types/types.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::attachAdmittedWall(
    semantic::Room                         *p_room_inout,
    geometric::Plane                       *p_candidateWall_in,
    const std::vector<geometric::Plane *>  &roomWalls_in,
    const std::vector<semantic::Passage *> &allPassages_in,
    const Eigen::Vector3d                  &roomViewpoint_world_m_in,
    const bool                              isCameraViewpoint_in)
{
    if (p_room_inout->setWalls(p_candidateWall_in) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setWalls returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    /* A wall that carries a passage never detaches a farther wall. */
    bool isPassageWall{false};
    for (semantic::Passage *p_passage : allPassages_in)
    {
        bool                            passageIsBad{true};
        std::vector<geometric::Plane *> associateWalls{};
        if (p_passage != nullptr &&
            p_passage->isBad(passageIsBad) ==
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS &&
            !passageIsBad &&
            p_passage->getAssociateWalls(associateWalls) ==
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS &&
            std::find(associateWalls.begin(),
                      associateWalls.end(),
                      p_candidateWall_in) != associateWalls.end())
        {
            isPassageWall = true;
            break;
        }
    }
    if (!p_sysParams->roomSeg.isSeeThroughGateEnabled ||
        !isCameraViewpoint_in || isPassageWall)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    for (geometric::Plane *p_fartherWall : roomWalls_in)
    {
        Eigen::Vector3d fartherCentroid_world_m{};
        if (p_fartherWall == nullptr || p_fartherWall == p_candidateWall_in ||
            p_fartherWall->getCentroid(fartherCentroid_world_m) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            continue;
        }
        const bool isSeenThroughCandidate =
            semantic_constraints::hasSeparatingFiniteWall(
                {p_candidateWall_in},
                roomViewpoint_world_m_in,
                fartherCentroid_world_m,
                static_cast<double>(p_sysParams->semSeg.reassociate
                                        .wallExtension.maximumInPlaneGap_m),
                false);
        bool wasWallRemoved{};
        if (!isSeenThroughCandidate ||
            p_room_inout->removeWall(p_fartherWall, wasWallRemoved) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            !wasWallRemoved)
        {
            continue;
        }
        int fartherWallId{}, candidateWallId{}, roomId{};
        if (p_fartherWall->getId(fartherWallId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            p_candidateWall_in->getId(candidateWallId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            p_room_inout->getId(roomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemMgr] Wall#" << fartherWallId << " detached from Room#"
                  << roomId << ": seen through nearer Wall#" << candidateWallId
                  << "." << std::endl;
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
