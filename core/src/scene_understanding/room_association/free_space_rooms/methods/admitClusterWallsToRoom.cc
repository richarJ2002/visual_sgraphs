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
 * @file            admitClusterWallsToRoom.cc
 *
 * @brief           Implements SemanticsManager::admitClusterWallsToRoom(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
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

SemanticsManagerStatus SemanticsManager::admitClusterWallsToRoom(
    semantic::Room                        *p_room_in,
    const std::vector<geometric::Plane *> &closestWalls_in,
    const Eigen::Vector3d                 &clusterCentroid_in)
{
    /* Find the walls of the room */
    std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
    if (p_room_in->getWalls(roomWalls) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /*!
     * @brief           Perspective guard: a wall observed through an
     *                  opening is on the far side of the passage's
     *                  supporting wall and therefore cannot bound this
     *                  room. enforcePassageApertureBackstop() decides and
     *                  applies the routing, the same check wall admission
     *                  uses, so cluster-driven and orphan-driven admission
     *                  route a far-side wall identically.
     */
    for (vs_graphs::core::geometric::Plane *p_wall : closestWalls_in)
    {
        /* Skip invalid walls */
        bool clusterWallIsBad{};
        if (!(p_wall == nullptr) &&
            p_wall->isBad(clusterWallIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_wall == nullptr || clusterWallIsBad)
        {
            continue;
        }

        /*!
         * @brief           Route far-side walls away from this room.
         *                  The ground normal is needed to project the
         *                  aperture crossing.
         */
        bool farSideBound = false;
        if (applyClusterWallBackstop(p_room_in, p_wall, farSideBound) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: applyClusterWallBackstop returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        if (farSideBound)
        {
            continue;
        }

        /* Check to see if closest wall is in any of the current rooms */
        bool alreadyInRoom = false;
        for (vs_graphs::core::geometric::Plane *p_existingWall : roomWalls)
        {
            int existingWallGetId{};
            if ((p_existingWall != nullptr) &&
                p_existingWall->getId(existingWallGetId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int wallGetId{};
            if ((p_existingWall != nullptr) &&
                p_wall->getId(wallGetId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_existingWall != nullptr && existingWallGetId == wallGetId)
            {
                alreadyInRoom = true;
                break;
            }
        }

        /* Skip walls already owned by a room. */
        if (alreadyInRoom)
        {
            continue;
        }

        /*
         * A mapped wall surface has one room owner. A room detected on
         * the opposite side must be bounded by its independently observed
         * wall surface rather than sharing this plane object.
         */
        semantic::Room *p_existingWallOwner = nullptr;
        if (findOtherWallOwner(p_room_in, p_wall, p_existingWallOwner) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findOtherWallOwner returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool               existingOwnerIsTransferableProvisional = false;
        semantic::Passage *p_transferPassage                      = nullptr;

        if (p_existingWallOwner != nullptr)
        {
            if (checkWallOwnerTransfer(p_existingWallOwner,
                                       p_wall,
                                       clusterCentroid_in,
                                       existingOwnerIsTransferableProvisional,
                                       p_transferPassage) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: checkWallOwnerTransfer returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if (!existingOwnerIsTransferableProvisional)
            {
                /* Never transfer a wall already owned by a confirmed room
                 * across a confirmed passage: a wall on the far side of an
                 * opening cannot bound the candidate room. Only a
                 * provisional single-wall structural element may hand its
                 * wall to the room that actually observes it. */
                continue;
            }
        }

        /* Reject wall hypotheses which would corrupt this boundary. */
        bool wasWallAdmitted{};
        if (admitWallToRoom(p_room_in, p_wall, wasWallAdmitted) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: admitWallToRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!wasWallAdmitted)
        {
            continue;
        }

        if (detachWallFromPreviousOwner(p_existingWallOwner,
                                        p_room_in,
                                        p_wall,
                                        existingOwnerIsTransferableProvisional,
                                        p_transferPassage) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: detachWallFromPreviousOwner returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        std::vector<geometric::Plane *> wallsAfterAdmission{};
        if (p_room_in->getWalls(wallsAfterAdmission) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        roomWalls = wallsAfterAdmission;

        /* Register the uniquely owned room-wall surface. */
        int registeredWallId{};
        if (p_wall->getId(registeredWallId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        vs_graphs::core::geometric::Plane *p_registeredRoomWall = nullptr;
        if (p_atlas->getRoomWallPlaneById(registeredWallId,
                                          p_registeredRoomWall) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomWallPlaneById returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_registeredRoomWall == nullptr)
        {
            if (p_atlas->addRoomWallPlane(p_wall) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addRoomWallPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
