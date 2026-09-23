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
        if (p_room == nullptr || p_room->isBad())
        {
            continue;
        }

        const std::vector<vs_graphs::core::geometric::Plane *> roomWalls =
            p_room->getWalls();
        if (roomWalls.empty())
        {
            continue;
        }

        Eigen::Vector3d wallCentroidSum = Eigen::Vector3d::Zero();
        int             wallCount       = 0;

        for (vs_graphs::core::geometric::Plane *p_wall : roomWalls)
        {
            if (p_wall == nullptr || p_wall->isBad())
            {
                continue;
            }

            const Eigen::Vector3d wallCentroid_World_m =
                p_wall->getCentroid().cast<double>();
            const std::optional<Eigen::Vector3d> inwardNormal_World =
                p_room->getWallNormalTowardRoom_World(p_wall);

            wallCentroidSum +=
                inwardNormal_World
                    ? wallCentroid_World_m +
                          centroidInwardOffset_m * (*inwardNormal_World)
                    : wallCentroid_World_m;
            wallCount++;
        }

        if (wallCount > 0)
        {
            p_room->setCentroid(wallCentroidSum / wallCount);
        }
    }
}

} // namespace core
} // namespace vs_graphs
