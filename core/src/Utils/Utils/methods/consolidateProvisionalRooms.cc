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
 * @file            consolidateProvisionalRooms.cc
 *
 * @brief           Implements Utils::consolidateProvisionalRooms(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

void Utils::consolidateProvisionalRooms(
    vs_graphs::core::semantic::Room *p_selectedRoom_inout,
    Atlas                           *p_atlas_inout)
{
    /* Confirm the selected room is valid */
    if (p_selectedRoom_inout == nullptr || p_selectedRoom_inout->isBad())
    {
        return;
    }

    /* Extract all walls assigned to the cluster-backed room */
    const std::vector<vs_graphs::core::geometric::Plane *> selectedWalls =
        p_selectedRoom_inout->getWalls();

    /* Create a set containing the selected wall IDs */
    std::unordered_set<int> selectedWallIds;
    selectedWallIds.reserve(selectedWalls.size());

    /* Insert every valid selected wall ID into the set */
    for (vs_graphs::core::geometric::Plane *wall : selectedWalls)
    {
        if (wall != nullptr && !wall->isBad())
        {
            selectedWallIds.insert(wall->getId());
        }
    }

    /* A room without walls cannot absorb another structural element */
    if (selectedWallIds.empty())
    {
        return;
    }

    /* Extract all rooms and provisional structural elements */
    const std::vector<vs_graphs::core::semantic::Room *> allRooms =
        p_atlas_inout->getAllRooms();

    /* Iterate through every possible redundant structural element */
    for (vs_graphs::core::semantic::Room *candidateRoom : allRooms)
    {
        /* Skip invalid rooms and the selected room itself */
        if (candidateRoom == nullptr || candidateRoom == p_selectedRoom_inout ||
            candidateRoom->isBad())
        {
            continue;
        }

        const std::vector<vs_graphs::core::semantic::Passage *> activePassages =
            p_atlas_inout->getAllPassages();
        const bool candidateIsLiveProspective = std::any_of(
            activePassages.begin(),
            activePassages.end(),
            [candidateRoom](vs_graphs::core::semantic::Passage *p_passage)
            {
                return p_passage != nullptr &&
                       p_passage->getProspectiveRoom() == candidateRoom;
            });

        if (candidateIsLiveProspective)
        {
            continue;
        }

        /*!
         * Only automatically absorb undefined structural elements.
         * Classified rooms and corridors must never be merged automatically.
         */
        if (candidateRoom->getRoomVariant() !=
            vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            continue;
        }

        /* Extract the candidate room walls */
        const std::vector<vs_graphs::core::geometric::Plane *> candidateWalls =
            candidateRoom->getWalls();

        /*!
         * Orphan-wall fallback creates one provisional SE per wall. Restrict
         * automatic consolidation to those single-wall provisional elements.
         * This prevents an emerging room on the opposite side of a shared wall
         * from being removed before it has enough evidence for classification.
         */
        std::vector<vs_graphs::core::geometric::Plane *> validCandidateWalls;
        validCandidateWalls.reserve(candidateWalls.size());

        for (vs_graphs::core::geometric::Plane *candidateWall : candidateWalls)
        {
            if (candidateWall != nullptr && !candidateWall->isBad())
            {
                validCandidateWalls.push_back(candidateWall);
            }
        }

        /* Only single-wall provisional elements are safe to absorb */
        if (validCandidateWalls.size() != 1)
        {
            continue;
        }

        /* Extract the single wall represented by the provisional element */
        vs_graphs::core::geometric::Plane *candidateWall =
            validCandidateWalls.front();

        /* The selected room must already contain the candidate wall */
        if (selectedWallIds.count(candidateWall->getId()) == 0)
        {
            continue;
        }

        /*!
         * Confirm the candidate centroid is still close to its wall plane.
         * This identifies a wall-centred orphan SE rather than a free-space
         * cluster which may represent a genuine room on the opposite side.
         */
        Eigen::Vector4d wallEquation =
            candidateWall->getGlobalEquation().coeffs();

        const double normalNorm = wallEquation.head<3>().norm();

        if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            continue;
        }

        wallEquation /= normalNorm;

        const double candidatePlaneDistance =
            std::abs(wallEquation.head<3>().dot(candidateRoom->getCentroid()) +
                     wallEquation(3));

        constexpr double provisionalWallDistanceThreshold = 0.25;

        if (candidatePlaneDistance > provisionalWallDistanceThreshold)
        {
            continue;
        }

        /* Preserve passage relationships before invalidating the candidate */
        const std::vector<vs_graphs::core::semantic::Passage *>
            candidatePassages = candidateRoom->getPassages();

        for (vs_graphs::core::semantic::Passage *passage : candidatePassages)
        {
            /* Skip invalid passages */
            if (passage == nullptr)
            {
                continue;
            }

            /* Check whether the selected room already contains the passage */
            const std::vector<vs_graphs::core::semantic::Passage *>
                selectedPassages = p_selectedRoom_inout->getPassages();

            const bool alreadyPresent = std::any_of(
                selectedPassages.begin(),
                selectedPassages.end(),
                [passage](vs_graphs::core::semantic::Passage *existingPassage)
                {
                    return existingPassage != nullptr &&
                           existingPassage->getId() == passage->getId();
                });

            /* Copy the passage relationship if required */
            if (!alreadyPresent)
            {
                p_selectedRoom_inout->setDoorways(passage);
            }
        }

        /* Preserve floor membership before retiring the provisional room. */
        for (vs_graphs::core::semantic::Floor *p_floor :
             p_atlas_inout->getAllFloors())
        {
            if (p_floor != nullptr)
            {
                p_floor->replaceRoom(candidateRoom, p_selectedRoom_inout);
            }
        }

        /* Mark the redundant provisional structural element as invalid */
        candidateRoom->setBad();
    }
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
