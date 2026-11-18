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
 * @file            resolveClusterRoom.cc
 *
 * @brief           Implements SemanticsManager::resolveClusterRoom(), declared
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

SemanticsManagerStatus SemanticsManager::resolveClusterRoom(
    std::size_t                            clusterId_in,
    const std::vector<Eigen::Vector3d>    &cluster_in,
    const Eigen::Vector3d                 &clusterCentroid_in,
    const std::vector<geometric::Plane *> &closestWalls_in,
    semantic::Room                        *p_clusterProspective_in,
    std::unordered_set<int>               &matchedRoomIds_inout,
    semantic::Room                       *&p_room_out)
{
    /* Prefer the passage's stable handle; otherwise use normal matching. */
    vs_graphs::core::semantic::Room *p_reusedClusterRoom = nullptr;
    if (!(p_clusterProspective_in != nullptr) &&
        associateRooms(clusterCentroid_in,
                       closestWalls_in,
                       cluster_in,
                       matchedRoomIds_inout,
                       p_reusedClusterRoom) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: associateRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    p_room_out = p_clusterProspective_in != nullptr ? p_clusterProspective_in
                                                    : p_reusedClusterRoom;

    /* Track matched room to prevent double-matching in this cycle */
    if (p_room_out != nullptr)
    {
        int roomId{};
        if (p_room_out->getId(roomId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        matchedRoomIds_inout.insert(roomId);
    }

    /*!
     * If no existing room describes this free-space cluster, try to reuse
     * a wall owner; otherwise defer creation to the bootstrap or a
     * passage-linked prospective room.
     *
     * Do not remove walls that are globally registered. Each mapped wall
     * surface remains owned by one room.
     *
     * Anti-duplicate guard: a wall already claimed by an existing room
     * means this cluster is the same open space (or the boundary-only
     * extension) of that room. Reusing the owner keeps the
     * one-wall-one-room invariant and prevents the classic two-room
     * duplicate derived from two walls of one open space. Only create a
     * fresh candidate when none of the cluster's walls is owned yet.
     */
    if (p_room_out == nullptr)
    {
        vs_graphs::core::semantic::Room *p_wallOwnerRoom = nullptr;

        std::vector<vs_graphs::core::semantic::Room *> existingRooms_world{};
        if (getAllRoomsInIdOrder(existingRooms_world) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllRoomsInIdOrder returned a failure "
                         "status; continuing as before.",
                         __func__);
        }

        for (vs_graphs::core::geometric::Plane *p_candidateWall :
             closestWalls_in)
        {
            bool candidateWallIsBad{};
            if (!(p_candidateWall == nullptr) &&
                p_candidateWall->isBad(candidateWallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_candidateWall == nullptr || candidateWallIsBad)
            {
                continue;
            }

            /* The owner of a face bounds the face's observed side; a
             * cluster on the other side is not that room. */
            bool isClusterOnForeignFaceSide{};
            if (isPointOnForeignSideOfWallFace(p_candidateWall,
                                               clusterCentroid_in,
                                               isClusterOnForeignFaceSide) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isPointOnForeignSideOfWallFace returned "
                             "a failure status although it cannot fail; "
                             "continuing as before.",
                             __func__);
            }
            if (isClusterOnForeignFaceSide)
            {
                continue;
            }

            for (vs_graphs::core::semantic::Room *p_existingRoom :
                 existingRooms_world)
            {
                bool existingRoomIsBad{};
                if (!(p_existingRoom == nullptr) &&
                    p_existingRoom->isBad(existingRoomIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isBad returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                int existingRoomId{};
                if (!(p_existingRoom == nullptr || existingRoomIsBad) &&
                    p_existingRoom->getId(existingRoomId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getId returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_existingRoom == nullptr || existingRoomIsBad ||
                    matchedRoomIds_inout.count(existingRoomId) > 0)
                {
                    continue;
                }

                std::vector<vs_graphs::core::geometric::Plane *>
                    roomWallsList{};
                if (p_existingRoom->getWalls(roomWallsList) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWalls returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }

                if (std::find(roomWallsList.begin(),
                              roomWallsList.end(),
                              p_candidateWall) != roomWallsList.end())
                {
                    p_wallOwnerRoom = p_existingRoom;
                    break;
                }
            }

            if (p_wallOwnerRoom != nullptr)
            {
                break;
            }
        }

        if (p_wallOwnerRoom != nullptr)
        {
            p_room_out = p_wallOwnerRoom;

            int reusedClusterRoomId{};
            if (p_room_out->getId(reusedClusterRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Reusing existing Room#"
                      << reusedClusterRoomId << " for cluster " << clusterId_in
                      << " (cluster walls already owned)." << std::endl;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
