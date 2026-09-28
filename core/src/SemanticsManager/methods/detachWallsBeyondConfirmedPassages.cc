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

#include "../private_functions.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::detachWallsBeyondConfirmedPassages(void)
{
    const types::SystemParams::RoomSeg::PassagePartition &partitionParameters =
        p_sysParams->roomSeg.passagePartition;

    if (!partitionParameters.enabled ||
        !partitionParameters.shouldDetachWallsBeyondPassages)
    {
        return;
    }

    geometric::Plane *p_groundPlane = p_atlas->getBiggestGroundPlane();

    if (p_groundPlane == nullptr || p_groundPlane->isBad())
    {
        return;
    }

    const Eigen::Vector4d groundEquation_World =
        p_groundPlane->getGlobalEquation().coeffs();
    const double groundNormalNorm = groundEquation_World.head<3>().norm();

    if (!groundEquation_World.allFinite() || groundNormalNorm < 1e-8)
    {
        return;
    }

    const Eigen::Vector3d groundNormal_World =
        groundEquation_World.head<3>() / groundNormalNorm;
    const double openingMargin_m =
        static_cast<double>(partitionParameters.openingMargin_m);
    const double minimumSideDistance_m = static_cast<double>(
        partitionParameters.wallCentroidMinimumSideDistance_m);

    std::vector<semantic::Passage *> confirmedOpenPassages;

    for (semantic::Passage *p_passage : p_atlas->getAllPassages())
    {
        bool passageIsPassable{};
        if ((p_passage != nullptr) &&
            p_passage->isPassable(passageIsPassable) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // isPassable cannot fail; continue as before.
        }
        if (p_passage != nullptr && passageIsPassable)
        {
            confirmedOpenPassages.push_back(p_passage);
        }
    }

    std::sort(
        confirmedOpenPassages.begin(),
        confirmedOpenPassages.end(),
        [](const semantic::Passage *p_first, const semantic::Passage *p_second)
        {
            int firstId{};
            if (p_first->getId(firstId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            int secondId{};
            if (p_second->getId(secondId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            return firstId < secondId;
        });

    if (confirmedOpenPassages.empty())
    {
        return;
    }

    const std::vector<semantic::Room *> allRooms = p_atlas->getAllRooms();

    for (semantic::Room *p_room : allRooms)
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

        Eigen::Vector3d roomCentroid_World_m{};
        if (p_room->getCentroid(roomCentroid_World_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getCentroid cannot fail; continue as before.
        }

        if (!roomCentroid_World_m.allFinite())
        {
            continue;
        }

        std::vector<geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }
        for (geometric::Plane *p_wall : roomWalls)
        {
            if (p_wall == nullptr || p_wall->isBad())
            {
                continue;
            }

            const Eigen::Vector3d wallCentroid_World_m =
                p_wall->getCentroid().cast<double>();

            if (!wallCentroid_World_m.allFinite())
            {
                continue;
            }

            semantic::Passage *p_separatingPassage = nullptr;

            for (semantic::Passage *p_passage : confirmedOpenPassages)
            {
                /*
                 * A passage's own supporting wall lies on its aperture plane,
                 * so it cannot satisfy the opposite-side distance test. This
                 * preserves the wall-passage relationship while rejecting a
                 * different wall reached only through that opening.
                 */
                if (segmentCrossesPassageOpening(roomCentroid_World_m,
                                                 wallCentroid_World_m,
                                                 p_passage,
                                                 groundNormal_World,
                                                 openingMargin_m,
                                                 minimumSideDistance_m))
                {
                    p_separatingPassage = p_passage;
                    break;
                }
            }

            if (p_separatingPassage == nullptr)
            {
                continue;
            }

            semantic::Room *p_confirmedOwner = nullptr;
            for (semantic::Room *p_otherRoom : allRooms)
            {
                bool otherRoomIsBad{};
                if (!(p_otherRoom == nullptr || p_otherRoom == p_room) &&
                    p_otherRoom->isBad(otherRoomIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // isBad cannot fail; continue as before.
                }
                semantic::Room::RoomVariant otherRoomRoomVariant{};
                if (!(p_otherRoom == nullptr || p_otherRoom == p_room ||
                      otherRoomIsBad) &&
                    p_otherRoom->getRoomVariant(otherRoomRoomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getRoomVariant cannot fail; continue as before.
                }
                if (p_otherRoom == nullptr || p_otherRoom == p_room ||
                    otherRoomIsBad ||
                    otherRoomRoomVariant ==
                        semantic::Room::RoomVariant::UNDEFINED)
                {
                    continue;
                }

                std::vector<geometric::Plane *> otherWalls{};
                if (p_otherRoom->getWalls(otherWalls) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getWalls cannot fail; continue as before.
                }
                if (std::find(otherWalls.begin(), otherWalls.end(), p_wall) !=
                    otherWalls.end())
                {
                    p_confirmedOwner = p_otherRoom;
                    break;
                }
            }

            semantic::Room *p_farSideRoom = nullptr;
            if (p_separatingPassage->getProspectiveRoom(p_farSideRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getProspectiveRoom cannot fail; continue as before.
            }
            bool farSideRoomIsBad{};
            if ((p_farSideRoom != nullptr) &&
                p_farSideRoom->isBad(farSideRoomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            if (p_farSideRoom != nullptr &&
                (farSideRoomIsBad || p_farSideRoom == p_room))
            {
                p_farSideRoom = nullptr;
            }

            bool roomWasWallRemoved{};
            if (p_room->removeWall(p_wall, roomWasWallRemoved) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                roomWasWallRemoved = false; // rejected input reads as before
            }
            if (!roomWasWallRemoved)
            {
                continue;
            }

            if (p_confirmedOwner != nullptr &&
                p_confirmedOwner != p_farSideRoom)
            {
                int roomId{};
                if (p_room->getId(roomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                int confirmedOwnerId{};
                if (p_confirmedOwner->getId(confirmedOwnerId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                std::cout
                    << "[SemMgr] Detached far-side Wall#" << p_wall->getId()
                    << " from semantic::Room#" << roomId
                    << "; retained distinct confirmed owner semantic::Room#"
                    << confirmedOwnerId << "." << std::endl;
                continue;
            }

            if (p_farSideRoom != nullptr)
            {
                if (p_farSideRoom->setWalls(p_wall) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // setWalls cannot fail; continue as before.
                }
                if (p_atlas->getRoomWallPlaneById(p_wall->getId()) == nullptr)
                {
                    p_atlas->addRoomWallPlane(p_wall);
                }
                int roomId2{};
                if (p_room->getId(roomId2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                int separatingPassageId{};
                if (p_separatingPassage->getId(separatingPassageId) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                int farSideRoomId{};
                if (p_farSideRoom->getId(farSideRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                std::cout << "[SemMgr] Redirected far-side Wall#"
                          << p_wall->getId() << " from semantic::Room#"
                          << roomId2 << " through semantic::Passage#"
                          << separatingPassageId << " to stable semantic::Room#"
                          << farSideRoomId << "." << std::endl;
                continue;
            }

            int roomId3{};
            if (p_room->getId(roomId3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            int separatingPassageId2{};
            if (p_separatingPassage->getId(separatingPassageId2) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            std::cout << "[SemMgr] Detached far-side Wall#" << p_wall->getId()
                      << " from semantic::Room#" << roomId3
                      << "; semantic::Passage#" << separatingPassageId2
                      << " has no stable far-side room, so the wall remains "
                         "orphaned."
                      << std::endl;
        }
    }
}

} // namespace core
} // namespace vs_graphs
