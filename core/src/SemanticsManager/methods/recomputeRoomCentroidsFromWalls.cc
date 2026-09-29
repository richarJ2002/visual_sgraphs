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

#include "SemanticsManager.h"

namespace vs_graphs
{
namespace core
{

void SemanticsManager::recomputeRoomCentroidsFromWalls(void)
{
    /* Same inward-nudge-then-average construction as
     * detectRoom_FreeSpaceCluster()'s wall-centroid correction -- see that
     * site's comment for why a plain mean of wall centroids is not
     * guaranteed to land inside the room. Kept in sync with it rather than
     * shared, since this function only runs for non-FREE_SPACE
     * roomSeg.method configurations (the FREE_SPACE path, this project's
     * configured default, uses the other site directly). */
    constexpr double centroidInwardOffset_m = 0.10;

    for (vs_graphs::core::semantic::Room *p_room : p_atlas->getAllRooms())
    {
        bool roomIsBad{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_room == nullptr || roomIsBad)
        {
            continue;
        }

        std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }
        if (roomWalls.empty())
        {
            continue;
        }

        Eigen::Vector3d wallCentroidSum = Eigen::Vector3d::Zero();
        int             wallCount       = 0;

        for (vs_graphs::core::geometric::Plane *p_wall : roomWalls)
        {
            bool wallIsBad{};
            if (!(p_wall == nullptr) &&
                p_wall->isBad(wallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            if (p_wall == nullptr || wallIsBad)
            {
                continue;
            }

            Eigen::Vector3d wallGetCentroid{};
            if (p_wall->getCentroid(wallGetCentroid) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
            }
            const Eigen::Vector3d wallCentroid_World_m =
                wallGetCentroid.cast<double>();
            std::optional<Eigen::Vector3d> inwardNormal_World{};
            if (p_room->getWallNormalTowardRoom_World(p_wall,
                                                      inwardNormal_World) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getWallNormalTowardRoom_World cannot fail; continue as
                // before.
            }

            wallCentroidSum +=
                inwardNormal_World
                    ? wallCentroid_World_m +
                          centroidInwardOffset_m * (*inwardNormal_World)
                    : wallCentroid_World_m;
            wallCount++;
        }

        if (wallCount > 0)
        {
            if (p_room->setCentroid(wallCentroidSum / wallCount) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setCentroid cannot fail; continue as before.
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
