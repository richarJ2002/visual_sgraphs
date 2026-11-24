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
 * @file            confirmClusterRoom.cc
 *
 * @brief           Implements SemanticsManager::confirmClusterRoom(), declared
 * in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/objects/SemanticsManagerClass.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::confirmClusterRoom(
    std::size_t                            clusterId_in,
    const std::vector<Eigen::Vector3d>    &cluster_in,
    semantic::Room                        *p_room_in,
    const std::vector<geometric::Plane *> &roomWalls_in,
    const std::vector<geometric::Plane *> &closestWalls_in,
    const FarSideProspectiveMatch         &prospectiveMatch_in,
    bool                                   roomIsPassageBoundProspective_in)
{
    /*!
     * Confirm the room from its free-space cluster.
     *
     * @note            A room is defined by connected free space, not by
     *                  having a particular arrangement or number of walls.
     *                  The associated walls describe the room boundary but
     *                  do not define whether the free-space region is a
     *                  room.
     */
    const bool validFreeSpaceCluster =
        cluster_in.size() >=
        static_cast<std::size_t>(p_sysParams->roomSeg.minClusterVertices);

    /*!
     * Require at least one associated wall before inserting the free-space
     * cluster into the structural hierarchy.
     */
    const bool hasBoundaryEvidence = !roomWalls_in.empty();

    bool prospectiveWallEvidenceStillMatches = false;
    if (prospectiveMatch_in.p_prospectiveRoom == p_room_in &&
        prospectiveMatch_in.p_passage != nullptr)
    {
        for (vs_graphs::core::geometric::Plane *p_wall :
             prospectiveMatch_in.prospectiveWallsBeforeCluster)
        {
            bool wallIsBad{};
            if ((p_wall != nullptr) &&
                p_wall->isBad(wallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_wall != nullptr && !wallIsBad &&
                std::find(closestWalls_in.begin(),
                          closestWalls_in.end(),
                          p_wall) != closestWalls_in.end() &&
                std::find(roomWalls_in.begin(), roomWalls_in.end(), p_wall) !=
                    roomWalls_in.end())
            {
                prospectiveWallEvidenceStillMatches = true;
                break;
            }
        }
    }

    /* Confirm the cluster-backed structural element as a room. */
    semantic::Room::RoomVariant roomVariant{};
    if ((validFreeSpaceCluster && hasBoundaryEvidence) &&
        p_room_in->getRoomVariant(roomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomVariant returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (validFreeSpaceCluster && hasBoundaryEvidence &&
        roomVariant == vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
    {
        if (prospectiveWallEvidenceStillMatches)
        {
            map::Map *p_roomMap = nullptr;
            if (p_room_in->getMap(p_roomMap) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_roomMap != nullptr)
            {
                unsigned long roomMapId{};
                if (p_roomMap->getId(roomMapId) !=
                    map::MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getId returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (promoteCandidateRoom(roomMapId, p_room_in) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: promoteCandidateRoom returned a "
                                 "failure status; continuing as before.",
                                 __func__);
                }
            }

            if (p_room_in->setRoomVariant(
                    vs_graphs::core::semantic::Room::RoomVariant::ROOM) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setRoomVariant returned a failure status although "
                    "it cannot fail; continuing as before.",
                    __func__);
            }
            int confirmedRoomId{};
            if (p_room_in->getId(confirmedRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room_in->setName("Room#" + std::to_string(confirmedRoomId)) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setName returned a failure status although it "
                    "cannot fail; continuing as before.",
                    __func__);
            }
            int confirmedCycleRoomId{};
            if (p_room_in->getId(confirmedCycleRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            prospectiveRoomCycles.erase(confirmedCycleRoomId);

            if (p_room_in->addPassageAssociation(
                    prospectiveMatch_in.p_passage) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addPassageAssociation returned a failure "
                             "status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (prospectiveMatch_in.p_passage->setProspectiveRoom(p_room_in) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            int promotedRoomId{};
            if (p_room_in->getId(promotedRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Promoted prospective Room#" << promotedRoomId
                      << " to ROOM from far-side cluster evidence."
                      << std::endl;
        }
        else if (!roomIsPassageBoundProspective_in)
        {
            map::Map *p_roomMap = nullptr;
            if (p_room_in->getMap(p_roomMap) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_roomMap != nullptr)
            {
                unsigned long roomMapId{};
                if (p_roomMap->getId(roomMapId) !=
                    map::MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getId returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (promoteCandidateRoom(roomMapId, p_room_in) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: promoteCandidateRoom returned a "
                                 "failure status; continuing as before.",
                                 __func__);
                }
            }
            if (p_room_in->setRoomVariant(
                    vs_graphs::core::semantic::Room::RoomVariant::ROOM) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setRoomVariant returned a failure status although "
                    "it cannot fail; continuing as before.",
                    __func__);
            }
            int prospectiveRoomId{};
            if (p_room_in->getId(prospectiveRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room_in->setName("Room#" +
                                   std::to_string(prospectiveRoomId)) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setName returned a failure status although it "
                    "cannot fail; continuing as before.",
                    __func__);
            }

            int structuralElementId{};
            if (p_room_in->getId(structuralElementId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Structural Element #" << structuralElementId
                      << " classified as a Room from free-space "
                         "cluster "
                      << clusterId_in << "." << std::endl;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
