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

UtilsStatus Utils::consolidateProvisionalRooms(
    vs_graphs::core::semantic::Room *p_selectedRoom_inout,
    Atlas                           *p_atlas_in)
{
    /* Confirm the selected room is valid */
    bool selectedRoom_inoutIsBad{};
    if (!(p_selectedRoom_inout == nullptr) &&
        p_selectedRoom_inout->isBad(selectedRoom_inoutIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // isBad cannot fail; continue as before.
    }
    if (p_selectedRoom_inout == nullptr || selectedRoom_inoutIsBad)
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    /* Extract all walls assigned to the cluster-backed room */
    std::vector<vs_graphs::core::geometric::Plane *> selectedWalls{};
    if (p_selectedRoom_inout->getWalls(selectedWalls) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getWalls cannot fail; continue as before.
    }

    /* Create a set containing the selected wall IDs */
    std::unordered_set<int> selectedWallIds;
    selectedWallIds.reserve(selectedWalls.size());

    /* Insert every valid selected wall ID into the set */
    for (vs_graphs::core::geometric::Plane *p_wall : selectedWalls)
    {
        if (p_wall != nullptr && !p_wall->isBad())
        {
            selectedWallIds.insert(p_wall->getId());
        }
    }

    /* A room without walls cannot absorb another structural element */
    if (selectedWallIds.empty())
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    /* Extract all rooms and provisional structural elements */
    const std::vector<vs_graphs::core::semantic::Room *> allRooms =
        p_atlas_in->getAllRooms();

    /* Iterate through every possible redundant structural element */
    for (vs_graphs::core::semantic::Room *p_candidateRoom : allRooms)
    {
        /* Skip invalid rooms and the selected room itself */
        bool candidateRoomIsBad{};
        if (!(p_candidateRoom == nullptr ||
              p_candidateRoom == p_selectedRoom_inout) &&
            p_candidateRoom->isBad(candidateRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_candidateRoom == nullptr ||
            p_candidateRoom == p_selectedRoom_inout || candidateRoomIsBad)
        {
            continue;
        }

        const std::vector<vs_graphs::core::semantic::Passage *> activePassages =
            p_atlas_in->getAllPassages();
        const bool candidateIsLiveProspective = std::any_of(
            activePassages.begin(),
            activePassages.end(),
            [p_candidateRoom](vs_graphs::core::semantic::Passage *p_passage)
            {
                vs_graphs::core::semantic::Room *p_passageProspectiveRoom =
                    nullptr;
                if ((p_passage != nullptr) &&
                    p_passage->getProspectiveRoom(p_passageProspectiveRoom) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getProspectiveRoom cannot fail; continue as before.
                }
                return p_passage != nullptr &&
                       p_passageProspectiveRoom == p_candidateRoom;
            });

        if (candidateIsLiveProspective)
        {
            continue;
        }

        /*!
         * Only automatically absorb undefined structural elements.
         * Classified rooms and corridors must never be merged automatically.
         */
        semantic::Room::RoomVariant candidateRoomRoomVariant{};
        if (p_candidateRoom->getRoomVariant(candidateRoomRoomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getRoomVariant cannot fail; continue as before.
        }
        if (candidateRoomRoomVariant !=
            vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            continue;
        }

        /* Extract the candidate room walls */
        std::vector<vs_graphs::core::geometric::Plane *> candidateWalls{};
        if (p_candidateRoom->getWalls(candidateWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }

        /*!
         * Orphan-wall fallback creates one provisional SE per wall. Restrict
         * automatic consolidation to those single-wall provisional elements.
         * This prevents an emerging room on the opposite side of a shared wall
         * from being removed before it has enough evidence for classification.
         */
        std::vector<vs_graphs::core::geometric::Plane *> validCandidateWalls;
        validCandidateWalls.reserve(candidateWalls.size());

        for (vs_graphs::core::geometric::Plane *p_candidateWall :
             candidateWalls)
        {
            if (p_candidateWall != nullptr && !p_candidateWall->isBad())
            {
                validCandidateWalls.push_back(p_candidateWall);
            }
        }

        /* Only single-wall provisional elements are safe to absorb */
        if (validCandidateWalls.size() != 1)
        {
            continue;
        }

        /* Extract the single wall represented by the provisional element */
        vs_graphs::core::geometric::Plane *p_candidateWall =
            validCandidateWalls.front();

        /* The selected room must already contain the candidate wall */
        if (selectedWallIds.count(p_candidateWall->getId()) == 0)
        {
            continue;
        }

        /*!
         * Confirm the candidate centroid is still close to its wall plane.
         * This identifies a wall-centred orphan SE rather than a free-space
         * cluster which may represent a genuine room on the opposite side.
         */
        Eigen::Vector4d wallEquation =
            p_candidateWall->getGlobalEquation().coeffs();

        const double normalNorm = wallEquation.head<3>().norm();

        if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            continue;
        }

        wallEquation /= normalNorm;

        Eigen::Vector3d candidateRoomCentroid{};
        if (p_candidateRoom->getCentroid(candidateRoomCentroid) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getCentroid cannot fail; continue as before.
        }
        const double candidatePlaneDistance =
            std::abs(wallEquation.head<3>().dot(candidateRoomCentroid) +
                     wallEquation(3));

        constexpr double provisionalWallDistanceThreshold = 0.25;

        if (candidatePlaneDistance > provisionalWallDistanceThreshold)
        {
            continue;
        }

        /* Preserve passage relationships before invalidating the candidate */
        std::vector<vs_graphs::core::semantic::Passage *> candidatePassages{};
        if (p_candidateRoom->getPassages(candidatePassages) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getPassages cannot fail; continue as before.
        }

        for (vs_graphs::core::semantic::Passage *p_candidatePassage :
             candidatePassages)
        {
            /* Skip invalid passages */
            if (p_candidatePassage == nullptr)
            {
                continue;
            }

            /* Check whether the selected room already contains the passage */
            std::vector<vs_graphs::core::semantic::Passage *>
                selectedPassages{};
            if (p_selectedRoom_inout->getPassages(selectedPassages) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getPassages cannot fail; continue as before.
            }

            const bool alreadyPresent = std::any_of(
                selectedPassages.begin(),
                selectedPassages.end(),
                [p_candidatePassage](
                    vs_graphs::core::semantic::Passage *p_existingPassage)
                {
                    int existingPassageId{};
                    if ((p_existingPassage != nullptr) &&
                        p_existingPassage->getId(existingPassageId) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    int candidatePassageId{};
                    if ((p_existingPassage != nullptr) &&
                        p_candidatePassage->getId(candidatePassageId) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    return p_existingPassage != nullptr &&
                           existingPassageId == candidatePassageId;
                });

            /* Copy the passage relationship if required */
            if (!alreadyPresent)
            {
                if (p_selectedRoom_inout->setDoorways(p_candidatePassage) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // setDoorways cannot fail; continue as before.
                }
            }
        }

        /* Preserve floor membership before retiring the provisional room. */
        for (vs_graphs::core::semantic::Floor *p_floor :
             p_atlas_in->getAllFloors())
        {
            if (p_floor != nullptr)
            {
                bool floorWasRoomReplaced{};
                if (p_floor->replaceRoom(p_candidateRoom,
                                         p_selectedRoom_inout,
                                         floorWasRoomReplaced) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    floorWasRoomReplaced =
                        false; // rejected input reads as before
                }
            }
        }

        /* Mark the redundant provisional structural element as invalid */
        if (p_candidateRoom->setBad() !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setBad cannot fail; continue as before.
        }
    }

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
