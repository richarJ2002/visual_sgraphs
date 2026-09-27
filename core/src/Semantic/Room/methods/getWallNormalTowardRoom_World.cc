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

#include "Semantic/Room.h"
#include <algorithm>
#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::optional<Eigen::Vector3d>
    Room::getWallNormalTowardRoom_World(const geometric::Plane *p_wall_in) const
{
    /* Reject a missing wall association. */
    if (p_wall_in == nullptr)
    {
        return std::nullopt;
    }

    /* Read, but never modify, the globally expressed wall equation. */
    Eigen::Vector4d wallEquation_World =
        p_wall_in->getGlobalEquation().coeffs();

    Eigen::Vector3d roomCentroid_World_m;

    {
        std::lock_guard<std::mutex> lock(mapMutex);
        roomCentroid_World_m = centroid;
    }

    /* Reject non-finite geometry before evaluating its signed distance. */
    if (!wallEquation_World.allFinite() || !roomCentroid_World_m.allFinite())
    {
        return std::nullopt;
    }

    const double wallNormalNorm = wallEquation_World.head<3>().norm();

    /* A plane without a usable normal has no defined orientation. */
    constexpr double minimumWallNormalNorm = 1e-8;

    if (!std::isfinite(wallNormalNorm) ||
        wallNormalNorm < minimumWallNormalNorm)
    {
        return std::nullopt;
    }

    /* Normalize all coefficients so the signed value is measured in metres. */
    wallEquation_World /= wallNormalNorm;

    Eigen::Vector3d wallNormalTowardRoom_World = wallEquation_World.head<3>();

    const double roomSignedDistanceToWall_m =
        wallNormalTowardRoom_World.dot(roomCentroid_World_m) +
        wallEquation_World(3);

    if (!std::isfinite(roomSignedDistanceToWall_m))
    {
        return std::nullopt;
    }

    /* Flip only the returned value when the stored normal points away. */
    if (roomSignedDistanceToWall_m < 0.0)
    {
        wallNormalTowardRoom_World *= -1.0;
    }

    return wallNormalTowardRoom_World;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
