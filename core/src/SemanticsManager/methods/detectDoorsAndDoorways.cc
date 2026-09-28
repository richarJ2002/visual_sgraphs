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

#include <cmath>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::detectDoorsAndDoorways(
    vs_graphs::core::Atlas *p_atlas_in)
{
    /* Confirm that the Atlas is valid */
    if (p_atlas_in == nullptr)
    {
        return;
    }

    /* Extract all planes from the current map */
    const std::vector<vs_graphs::core::geometric::Plane *> allPlanes =
        p_atlas_in->getAllPlanes();

    /* Initialise lists of valid wall and door planes */
    std::vector<vs_graphs::core::geometric::Plane *> wallPlanes;
    std::vector<vs_graphs::core::geometric::Plane *> doorPlanes;

    wallPlanes.reserve(allPlanes.size());
    doorPlanes.reserve(allPlanes.size());

    /* Separate mapped planes according to their confirmed semantic type */
    for (vs_graphs::core::geometric::Plane *p_plane : allPlanes)
    {
        /* Skip invalid mapped planes */
        if (p_plane == nullptr || p_plane->isBad())
        {
            continue;
        }

        /* Store confirmed wall planes */
        if (p_plane->getPlaneType() ==
            vs_graphs::core::geometric::Plane::PlaneVariant::WALL)
        {
            wallPlanes.push_back(p_plane);
            continue;
        }

        /* Store confirmed door planes */
        if (p_plane->getPlaneType() ==
            vs_graphs::core::geometric::Plane::PlaneVariant::DOOR)
        {
            doorPlanes.push_back(p_plane);
        }
    }

    /* Filter wall planes to those with sufficient observations.
     * Provisional rooms are valid evidence - don't require CONFIRMED room
     * association. */
    std::vector<vs_graphs::core::geometric::Plane *> confirmedWallPlanes;
    confirmedWallPlanes.reserve(wallPlanes.size());

    for (vs_graphs::core::geometric::Plane *p_wall : wallPlanes)
    {
        if (p_wall == nullptr || p_wall->isBad())
        {
            continue;
        }

        // Quality gate: minimum observations, not room confirmation status
        if (p_wall->getObservationCount() >=
            p_sysParams->roomSeg.minimumWallObservationCount)
        {
            confirmedWallPlanes.push_back(p_wall);
        }
        else
        {
            std::cout << "[SemMgr] Skipping wall " << p_wall->getId()
                      << " for passage detection: insufficient observations ("
                      << p_wall->getObservationCount() << " < "
                      << p_sysParams->roomSeg.minimumWallObservationCount
                      << ")." << std::endl;
        }
    }

    /*  Detect blocked passages represented by closed semantic door planes */
    for (vs_graphs::core::geometric::Plane *p_door : doorPlanes)
    {
        /* Skip invalid door planes */
        if (p_door == nullptr || p_door->isBad())
        {
            continue;
        }

        for (vs_graphs::core::geometric::Plane *p_wall : confirmedWallPlanes)
        {
            /* Skip invalid wall planes */
            if (p_wall == nullptr || p_wall->isBad())
            {
                continue;
            }

            /* Door and wall must be parallel */
            if (!utils::utils::Utils::arePlanesParallel(p_door, p_wall))
            {
                continue;
            }

            /* Door must lie close to the supporting wall */
            if (utils::utils::Utils::arePlanesApartEnough(
                    p_door,
                    p_wall,
                    p_sysParams->semSeg.maxWallDoorDistance))
            {
                continue;
            }

            /* Create a blocked passage associated with the supporting wall */
            GeoSemHelpers::createMapPassage(p_atlas, p_door, p_wall, false);
        }
    }

    /*!
     * Detect open passages from connected Voxblox skeleton edges which breach
     * finite mapped wall surfaces.
     *
     * @note        Camera trajectory crossings are deliberately not used.
     */
    detectOpenPassagesFromSkeletonEdges(confirmedWallPlanes);
}

} // namespace core
} // namespace vs_graphs
