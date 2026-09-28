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
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNSS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#include "GeoSemHelpers.h"

namespace vs_graphs
{
namespace core
{

void GeoSemHelpers::associateGroundPlaneToRoom(
    Atlas                           *p_atlas_in,
    vs_graphs::core::semantic::Room *p_givenRoom_inout)
{
    std::vector<vs_graphs::core::geometric::Plane *> allWalls{};
    if (p_givenRoom_inout->getWalls(allWalls) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getWalls cannot fail; continue as before.
    }
    vs_graphs::core::geometric::Plane *p_associatedGroundPlane = nullptr;
    size_t                             maximumInliers          = 0;

    // get the ground planes from the Atlas
    std::vector<vs_graphs::core::geometric::Plane *> groundPlanes;
    for (const auto &plane : p_atlas_in->getAllPlanes())
    {
        if (plane->getPlaneType() ==
            vs_graphs::core::geometric::Plane::PlaneVariant::GROUND)
        {
            groundPlanes.push_back(plane);
        }
    }

    if (groundPlanes.empty())
    {
        // no ground planes in the Atlas
        return;
    }
    else
    {
        // check which ground plane has the most points within the walls
        for (const auto &plane : groundPlanes)
        {
            // count inliers of the plane
            size_t inliers = countGroundPlanePointsWithinWalls(allWalls, plane);

            // update the associated ground plane if the current plane has more
            // inliers
            if (inliers > maximumInliers)
            {
                maximumInliers          = inliers;
                p_associatedGroundPlane = plane;
            }
        }

        if (p_associatedGroundPlane != nullptr)
        {
            if (p_givenRoom_inout->setGroundPlane(p_associatedGroundPlane) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setGroundPlane cannot fail; continue as before.
            }
        }
        else
        {
            // set the biggest ground plane as the ground plane of the room
            // [TODO] - logic for when ground plane is not found within the
            // walls
            if (p_givenRoom_inout->setGroundPlane(
                    p_atlas_in->getBiggestGroundPlane()) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setGroundPlane cannot fail; continue as before.
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
