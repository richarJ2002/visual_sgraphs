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
 * @file            repairRoomWallClashes.cc
 *
 * @brief           Implements repairRoomWallClashes(), declared in
 *                  room_validation/private_functions/private_functions.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/room_validation/private_functions/private_functions.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

#include <algorithm>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus repairRoomWallClashes(
    semantic::Room                                       *p_room_inout,
    const Eigen::Vector3d                                &groundNormal_world_in,
    const Eigen::Vector3d                                &groundAxisU_world_in,
    const Eigen::Vector3d                                &groundAxisV_world_in,
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters_in,
    std::vector<FiniteWallSegment2d>                     &wallSegments_out,
    bool &hasAmbiguousConflict_out)
{
    bool boundaryWasRepaired = true;

    while (boundaryWasRepaired)
    {
        boundaryWasRepaired      = false;
        hasAmbiguousConflict_out = false;
        wallSegments_out.clear();

        std::vector<geometric::Plane *> roomWalls2{};
        if (p_room_inout->getWalls(roomWalls2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_wall : roomWalls2)
        {
            FiniteWallSegment2d wallSegment;

            bool isBuilt{};
            if (buildFiniteWallSegment2d(
                    p_wall,
                    groundNormal_world_in,
                    groundAxisU_world_in,
                    groundAxisV_world_in,
                    topologyParameters_in.endpointTrimRatio,
                    topologyParameters_in.minimumWallLength_m,
                    wallSegment,
                    isBuilt) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: buildFiniteWallSegment2d returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            if (isBuilt)
            {
                wallSegments_out.push_back(std::move(wallSegment));
            }
        }

        int roomId{};
        if (p_room_inout->getId(roomId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<geometric::Plane *> roomWalls3{};
        if (p_room_inout->getWalls(roomWalls3) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemMgr] Room#" << roomId
                  << " boundary check: roomWalls=" << roomWalls3.size()
                  << ", wallSegments_out=" << wallSegments_out.size()
                  << ", minWallCount=" << topologyParameters_in.minimumWallCount
                  << std::endl;

        if (wallSegments_out.size() < topologyParameters_in.minimumWallCount)
        {
            if (updateRoomBoundaryStatus(
                    p_room_inout,
                    semantic::Room::BoundaryStatus::INCOMPLETE) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateRoomBoundaryStatus returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            continue;
        }

        for (std::size_t firstWallIndex = 0U;
             firstWallIndex < wallSegments_out.size() && !boundaryWasRepaired &&
             !hasAmbiguousConflict_out;
             ++firstWallIndex)
        {
            for (std::size_t secondWallIndex = firstWallIndex + 1U;
                 secondWallIndex < wallSegments_out.size();
                 ++secondWallIndex)
            {
                Eigen::Vector2d lineIntersection_world_m;
                double          firstParameter  = 0.0;
                double          secondParameter = 0.0;

                bool hasIntersection{};
                if (intersectSupportingLines(wallSegments_out[firstWallIndex],
                                             wallSegments_out[secondWallIndex],
                                             lineIntersection_world_m,
                                             firstParameter,
                                             secondParameter,
                                             hasIntersection) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: intersectSupportingLines returned a "
                                 "failure status although it cannot fail; "
                                 "continuing as before.",
                                 __func__);
                }
                if (!hasIntersection || firstParameter < 0.0 ||
                    firstParameter > 1.0 || secondParameter < 0.0 ||
                    secondParameter > 1.0)
                {
                    continue;
                }

                const double firstInteriorDistance_m =
                    std::min(firstParameter, 1.0 - firstParameter) *
                    wallSegments_out[firstWallIndex].length_m;
                const double secondInteriorDistance_m =
                    std::min(secondParameter, 1.0 - secondParameter) *
                    wallSegments_out[secondWallIndex].length_m;

                if (std::max(firstInteriorDistance_m,
                             secondInteriorDistance_m) <=
                    topologyParameters_in.maximumInteriorIntersection_m)
                {
                    continue;
                }

                const double firstSupport =
                    wallSegments_out[firstWallIndex].supportScore;
                const double secondSupport =
                    wallSegments_out[secondWallIndex].supportScore;
                const double weakerSupport =
                    std::max(std::min(firstSupport, secondSupport), 1e-8);
                const double supportRatio =
                    std::max(firstSupport, secondSupport) / weakerSupport;

                if (supportRatio <
                    topologyParameters_in.decisiveConflictSupportRatio)
                {
                    hasAmbiguousConflict_out = true;
                    break;
                }

                geometric::Plane *p_rejectedWall =
                    firstSupport < secondSupport
                        ? wallSegments_out[firstWallIndex].p_wall
                        : wallSegments_out[secondWallIndex].p_wall;
                geometric::Plane *p_retainedWall =
                    firstSupport < secondSupport
                        ? wallSegments_out[secondWallIndex].p_wall
                        : wallSegments_out[firstWallIndex].p_wall;

                bool roomWasWallRemoved{};
                if (p_room_inout->removeWall(p_rejectedWall,
                                             roomWasWallRemoved) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    roomWasWallRemoved = false;
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: removeWall rejected its input; "
                                "continuing as before.",
                                __func__);
                }
                if (roomWasWallRemoved)
                {
                    boundaryWasRepaired = true;

                    int roomId2{};
                    if (p_room_inout->getId(roomId2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    int rejectedWallGetId{};
                    if (p_rejectedWall->getId(rejectedWallGetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    int retainedWallGetId{};
                    if (p_retainedWall->getId(retainedWallGetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    std::cout << "[SemMgr] Detached clashing Wall#"
                              << rejectedWallGetId << " from Room#" << roomId2
                              << "; Wall#" << retainedWallGetId
                              << " has decisively stronger finite support."
                              << std::endl;
                }

                break;
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
