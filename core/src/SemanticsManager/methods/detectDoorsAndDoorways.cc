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

void SemanticsManager::detectDoorsAndDoorways(vs_graphs::core::Atlas *pAtlas)
{
    /* Confirm that the Atlas is valid */
    if (pAtlas == nullptr)
    {
        return;
    }

    /* Extract all planes from the current map */
    const std::vector<vs_graphs::core::geometric::Plane *> allPlanes =
        pAtlas->getAllPlanes();

    /* Initialise lists of valid wall and door planes */
    std::vector<vs_graphs::core::geometric::Plane *> wallPlanes;
    std::vector<vs_graphs::core::geometric::Plane *> doorPlanes;

    wallPlanes.reserve(allPlanes.size());
    doorPlanes.reserve(allPlanes.size());

    /* Separate mapped planes according to their confirmed semantic type */
    for (vs_graphs::core::geometric::Plane *plane : allPlanes)
    {
        /* Skip invalid mapped planes */
        if (plane == nullptr || plane->isBad())
        {
            continue;
        }

        /* Store confirmed wall planes */
        if (plane->getPlaneType() ==
            vs_graphs::core::geometric::Plane::PlaneVariant::WALL)
        {
            wallPlanes.push_back(plane);
            continue;
        }

        /* Store confirmed door planes */
        if (plane->getPlaneType() ==
            vs_graphs::core::geometric::Plane::PlaneVariant::DOOR)
        {
            doorPlanes.push_back(plane);
        }
    }

    /* Filter wall planes to those with sufficient observations.
     * Provisional rooms are valid evidence - don't require CONFIRMED room
     * association. */
    std::vector<vs_graphs::core::geometric::Plane *> confirmedWallPlanes;
    confirmedWallPlanes.reserve(wallPlanes.size());

    for (vs_graphs::core::geometric::Plane *wall : wallPlanes)
    {
        if (wall == nullptr || wall->isBad())
        {
            continue;
        }

        // Quality gate: minimum observations, not room confirmation status
        if (wall->getObservationCount() >=
            p_sysParams->roomSeg.minimumWallObservationCount)
        {
            confirmedWallPlanes.push_back(wall);
        }
        else
        {
            std::cout << "[SemMgr] Skipping wall " << wall->getId()
                      << " for passage detection: insufficient observations ("
                      << wall->getObservationCount() << " < "
                      << p_sysParams->roomSeg.minimumWallObservationCount
                      << ")." << std::endl;
        }
    }

    /*  Detect blocked passages represented by closed semantic door planes */
    for (vs_graphs::core::geometric::Plane *door : doorPlanes)
    {
        /* Skip invalid door planes */
        if (door == nullptr || door->isBad())
        {
            continue;
        }

        for (vs_graphs::core::geometric::Plane *wall : confirmedWallPlanes)
        {
            /* Skip invalid wall planes */
            if (wall == nullptr || wall->isBad())
            {
                continue;
            }

            /* Door and wall must be parallel */
            if (!utils::utils::Utils::arePlanesParallel(door, wall))
            {
                continue;
            }

            /* Door must lie close to the supporting wall */
            if (utils::utils::Utils::arePlanesApartEnough(
                    door,
                    wall,
                    p_sysParams->semSeg.maxWallDoorDistance))
            {
                continue;
            }

            /* Create a blocked passage associated with the supporting wall */
            GeoSemHelpers::createMapPassage(p_atlas, door, wall, false);
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
