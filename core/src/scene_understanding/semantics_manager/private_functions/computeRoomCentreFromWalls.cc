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
 * @file            computeRoomCentreFromWalls.cc
 *
 * @brief           Implements computeRoomCentreFromWalls(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantic/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "private_functions.h"

#include <cstddef>
#include <vector>

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus computeRoomCentreFromWalls(
    semantic::Room                        *p_room_in,
    const std::vector<geometric::Plane *> &roomWalls_in,
    std::optional<Eigen::Vector3d>        &roomCentre_world_m_out)
{
    roomCentre_world_m_out.reset();
    if (p_room_in == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /*!
     * Fallback centre, and the centre's height in every case: the mean of
     * the wall centroids, each pushed inward along its room-facing normal.
     * A plain mean is not guaranteed to land inside a partly observed room
     * (two adjacent walls put it on the shared corner); pushing each point
     * inward first keeps every contributing point, and so their mean,
     * inside.
     */
    constexpr double centroidInwardOffset_m    = 0.10;
    Eigen::Vector3d  nudgedCentroidSum_world_m = Eigen::Vector3d::Zero();
    std::size_t      validWallCount            = 0U;
    for (geometric::Plane *p_wall : roomWalls_in)
    {
        bool wallIsBad{};
        if (p_wall != nullptr &&
            p_wall->isBad(wallIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_wall == nullptr || wallIsBad)
        {
            continue;
        }

        Eigen::Vector3d wallCentroid_world_m{};
        if (p_wall->getCentroid(wallCentroid_world_m) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (!wallCentroid_world_m.allFinite())
        {
            continue;
        }

        std::optional<Eigen::Vector3d> inwardNormal_world{};
        if (p_room_in->getWallNormalTowardRoom_world(p_wall,
                                                     inwardNormal_world) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWallNormalTowardRoom_world returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        nudgedCentroidSum_world_m +=
            inwardNormal_world
                ? wallCentroid_world_m +
                      centroidInwardOffset_m * (*inwardNormal_world)
                : wallCentroid_world_m;
        validWallCount++;
    }
    if (validWallCount == 0U)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    const Eigen::Vector3d nudgedCentroidMean_world_m =
        nudgedCentroidSum_world_m / static_cast<double>(validWallCount);

    /* The floor plane is the one perpendicular to the room's map up. */
    Eigen::Vector3d upDirection_world = Eigen::Vector3d::Zero();
    map::Map       *p_roomMap         = nullptr;
    if (p_room_in->getMap(p_roomMap) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool isUpFromImuGravity{};
    if (p_roomMap != nullptr &&
        p_roomMap->getUpDirection(upDirection_world, isUpFromImuGravity) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getUpDirection returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    RoomWallFootprint wallFootprint{};
    bool              hasWallFootprint{};
    if (computeRoomWallFootprint(roomWalls_in,
                                 upDirection_world,
                                 wallFootprint,
                                 hasWallFootprint) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeRoomWallFootprint returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!hasWallFootprint)
    {
        roomCentre_world_m_out = nudgedCentroidMean_world_m;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Box centre on the floor plane, fallback height along up. The three
     * axes are orthonormal, so the components recombine directly. */
    const Eigen::Vector3d unitUp_world = upDirection_world.normalized();
    roomCentre_world_m_out =
        0.5 * (wallFootprint.minimumFirst_m + wallFootprint.maximumFirst_m) *
            wallFootprint.firstAxis_world +
        0.5 * (wallFootprint.minimumSecond_m + wallFootprint.maximumSecond_m) *
            wallFootprint.secondAxis_world +
        nudgedCentroidMean_world_m.dot(unitUp_world) * unitUp_world;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
