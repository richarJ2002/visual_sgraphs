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

#include "Geometric/Plane.h"
#include "Geometric/PlaneStatus.h"
#include "Semantic/Room.h"
#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::getWallNormalTowardRoom_World(
    const geometric::Plane         *p_wall_in,
    std::optional<Eigen::Vector3d> &wallNormalTowardRoom_World_out) const
{
    /* Reject a missing wall association. */
    if (p_wall_in == nullptr)
    {
        wallNormalTowardRoom_World_out = std::nullopt;
        return RoomStatus::ROOM_STATUS_SUCCESS;
    }

    /* Read, but never modify, the globally expressed wall equation. */
    g2o::Plane3D wallGetGlobalEquation{};
    if (p_wall_in->getGlobalEquation(wallGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d wallEquation_World = wallGetGlobalEquation.coeffs();

    Eigen::Vector3d roomCentroid_World_m;

    {
        std::lock_guard<std::mutex> lock(mapMutex);
        roomCentroid_World_m = centroid;
    }

    /* Reject non-finite geometry before evaluating its signed distance. */
    if (!wallEquation_World.allFinite() || !roomCentroid_World_m.allFinite())
    {
        wallNormalTowardRoom_World_out = std::nullopt;
        return RoomStatus::ROOM_STATUS_SUCCESS;
    }

    const double wallNormalNorm = wallEquation_World.head<3>().norm();

    /* A plane without a usable normal has no defined orientation. */
    constexpr double minimumWallNormalNorm = 1e-8;

    if (!std::isfinite(wallNormalNorm) ||
        wallNormalNorm < minimumWallNormalNorm)
    {
        wallNormalTowardRoom_World_out = std::nullopt;
        return RoomStatus::ROOM_STATUS_SUCCESS;
    }

    /* Normalize all coefficients so the signed value is measured in metres. */
    wallEquation_World /= wallNormalNorm;

    Eigen::Vector3d wallNormalTowardRoom_World = wallEquation_World.head<3>();

    const double roomSignedDistanceToWall_m =
        wallNormalTowardRoom_World.dot(roomCentroid_World_m) +
        wallEquation_World(3);

    if (!std::isfinite(roomSignedDistanceToWall_m))
    {
        wallNormalTowardRoom_World_out = std::nullopt;
        return RoomStatus::ROOM_STATUS_SUCCESS;
    }

    /* Flip only the returned value when the stored normal points away. */
    if (roomSignedDistanceToWall_m < 0.0)
    {
        wallNormalTowardRoom_World *= -1.0;
    }

    wallNormalTowardRoom_World_out = wallNormalTowardRoom_World;
    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
