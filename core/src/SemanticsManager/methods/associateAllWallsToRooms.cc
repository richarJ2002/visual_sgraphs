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

void SemanticsManager::associateAllWallsToRooms(void)
{
    /*!
     * A wall must be observed from several keyframes before it is inserted into
     * the higher-level semantic hierarchy.
     */
    /* Extract all planes from the current map */
    const std::vector<vs_graphs::core::geometric::Plane *> allPlanes =
        p_atlas->getAllPlanes();

    /* Extract all current rooms and provisional structural elements */
    std::vector<vs_graphs::core::semantic::Room *> allRooms =
        p_atlas->getAllRooms();
    Map            *p_activeMap           = p_atlas->getCurrentMap();
    semantic::Room *p_currentRoom         = nullptr;
    const int       currentRoomIdSnapshot = getCurrentRoomId();
    const auto      planeClassName =
        [](const geometric::Plane::PlaneVariant variant_in)
    {
        switch (variant_in)
        {
        case geometric::Plane::PlaneVariant::WALL:
            return "WALL";
        case geometric::Plane::PlaneVariant::GROUND:
            return "GROUND";
        case geometric::Plane::PlaneVariant::DOOR:
            return "DOOR";
        case geometric::Plane::PlaneVariant::WINDOW:
            return "WINDOW";
        case geometric::Plane::PlaneVariant::UNDEFINED:
        default:
            return "UNDEFINED";
        }
    };
    for (semantic::Room *p_room : allRooms)
    {
        if (p_room != nullptr && !p_room->isBad() &&
            p_room->getMap() == p_activeMap &&
            p_room->getId() == currentRoomIdSnapshot &&
            p_room->getRoomVariant() == semantic::Room::RoomVariant::ROOM)
        {
            p_currentRoom = p_room;
            break;
        }
    }

    /* Ground-aligned axes for evaluateWallAdmissionEvidence's height/width
     * gate (see the comment at its definition). */
    geometric::Plane *p_groundPlaneForEvidence =
        p_atlas->getBiggestGroundPlane();
    Eigen::Vector3d groundNormalForEvidence_World = Eigen::Vector3d::Zero();
    if (p_groundPlaneForEvidence != nullptr &&
        !p_groundPlaneForEvidence->isBad())
    {
        const Eigen::Vector4d groundEquation =
            p_groundPlaneForEvidence->getGlobalEquation().coeffs();
        const double groundEquationNormalNorm = groundEquation.head<3>().norm();
        if (groundEquation.allFinite() && groundEquationNormalNorm > 1e-8)
        {
            groundNormalForEvidence_World =
                groundEquation.head<3>() / groundEquationNormalNorm;
        }
    }

    /* Helper which confirms that a room already contains a wall */
    const auto roomContainsWall =
        [](vs_graphs::core::semantic::Room   *room,
           vs_graphs::core::geometric::Plane *p_wall) -> bool
    {
        /* Return false if either object is invalid */
        if (room == nullptr || p_wall == nullptr)
        {
            return false;
        }

        /* Extract the walls currently assigned to the room */
        const std::vector<vs_graphs::core::geometric::Plane *> roomWalls =
            room->getWalls();

        /* Check whether the requested wall is already present */
        return std::any_of(
            roomWalls.begin(),
            roomWalls.end(),
            [p_wall](vs_graphs::core::geometric::Plane *p_existingWall)
            {
                return p_existingWall != nullptr &&
                       p_existingWall->getId() == p_wall->getId();
            });
    };

    /* Iterate through every mapped plane */
    for (vs_graphs::core::geometric::Plane *p_wall : allPlanes)
    {
        /* Skip invalid planes */
        if (p_wall == nullptr || p_wall->isBad())
        {
            continue;
        }

        /* Only fitted walls with semantic and observation evidence enter. */
        const WallAdmissionEvidence admissionEvidence =
            evaluateWallAdmissionEvidence(p_wall,
                                          p_sysParams,
                                          groundNormalForEvidence_World);
        if (!admissionEvidence.isAdmissible)
        {
            const std::string reason =
                p_wall->getPlaneType() !=
                            geometric::Plane::PlaneVariant::WALL ||
                        p_wall->getExpectedPlaneType() !=
                            geometric::Plane::PlaneVariant::WALL
                    ? "CLASS_NOT_WALL"
                : !admissionEvidence.hasAdequateFiniteFit
                    ? "INADEQUATE_FINITE_FIT"
                    : "INSUFFICIENT_OBSERVATIONS";
            if (loggedWallRejectionReasons[p_wall->getId()] != reason)
            {
                loggedWallRejectionReasons[p_wall->getId()] = reason;
                std::cout << "SG_PIPELINE {\"event\":\"wall_rejection\","
                             "\"map_id\":"
                          << (p_activeMap != nullptr
                                  ? static_cast<long long>(p_activeMap->getId())
                                  : -1)
                          << ",\"semantic_cycle\":" << pipelineSemanticCycle
                          << ",\"wall_id\":" << p_wall->getId()
                          << ",\"class\":\""
                          << planeClassName(p_wall->getPlaneType())
                          << "\",\"lifecycle\":\"REJECTED\","
                             "\"owner\":\"NONE\",\"reason\":\""
                          << reason << "\",\"support\":"
                          << admissionEvidence.fittedPointCount
                          << ",\"observations\":"
                          << admissionEvidence.observationCount << "}"
                          << std::endl;
            }
            continue;
        }
        loggedWallRejectionReasons.erase(p_wall->getId());

        /* Init flag which confirms whether the wall already has a parent */
        bool wallHasRoom = false;

        /* Search every valid room for the wall */
        for (vs_graphs::core::semantic::Room *room : allRooms)
        {
            /* Skip invalid rooms */
            if (room == nullptr || room->isBad())
            {
                continue;
            }

            /* If the room contains the wall, the hierarchy is complete */
            if (roomContainsWall(room, p_wall))
            {
                wallHasRoom = true;
                break;
            }
        }

        /* Skip walls which already belong to a room */
        if (wallHasRoom)
        {
            continue;
        }

        const bool admitted =
            p_currentRoom != nullptr && admitWallToRoom(p_currentRoom, p_wall);
        semantic::Room *p_selectedOwner = nullptr;
        for (semantic::Room *p_room : p_atlas->getAllRooms())
        {
            if (p_room != nullptr && !p_room->isBad() &&
                roomContainsWall(p_room, p_wall))
            {
                p_selectedOwner = p_room;
                break;
            }
        }

        if (admitted && p_selectedOwner != nullptr)
        {
            if (p_atlas->getRoomWallPlaneById(p_wall->getId()) == nullptr)
            {
                p_atlas->addRoomWallPlane(p_wall);
            }
            undefendedWalls.erase(p_wall->getId());
            loggedOrphanWallIds.erase(p_wall->getId());
            std::cout << "SG_PIPELINE {\"event\":\"wall_admission\","
                         "\"map_id\":"
                      << p_activeMap->getId()
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle
                      << ",\"wall_id\":" << p_wall->getId()
                      << ",\"class\":\"WALL\","
                         "\"lifecycle\":\"COMMITTED\",\"owner_room_id\":"
                      << p_selectedOwner->getId() << ",\"reason\":\""
                      << (p_selectedOwner == p_currentRoom
                              ? "CURRENT_ROOM_OBSERVATION"
                              : "PASSAGE_FAR_SIDE_PRECEDENCE")
                      << "\",\"support\":" << admissionEvidence.fittedPointCount
                      << ",\"observations\":"
                      << admissionEvidence.observationCount << "}" << std::endl;
            continue;
        }

        /*!
         * The free-space detector did not assign this wall to a room.
         * Register the wall for future association via passages or room
         * detection. Do NOT create a provisional room for orphan walls - only
         * passages create prospective rooms.
         */

        /* Register the uniquely owned wall in the room-wall collection. */
        if (p_atlas->getRoomWallPlaneById(p_wall->getId()) == nullptr)
        {
            p_atlas->addRoomWallPlane(p_wall);
        }

        if (loggedOrphanWallIds.insert(p_wall->getId()).second)
        {
            std::cout << "SG_PIPELINE {\"event\":\"wall_pending\","
                         "\"map_id\":"
                      << (p_activeMap != nullptr
                              ? static_cast<long long>(p_activeMap->getId())
                              : -1)
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle
                      << ",\"wall_id\":" << p_wall->getId()
                      << ",\"class\":\"WALL\","
                         "\"lifecycle\":\"PENDING\",\"owner\":\"PENDING\","
                         "\"reason\":\""
                      << (p_currentRoom == nullptr ? "NO_CURRENT_ROOM"
                                                   : "SAFE_ADMISSION_REJECTED")
                      << "\",\"support\":" << admissionEvidence.fittedPointCount
                      << ",\"observations\":"
                      << admissionEvidence.observationCount
                      << ",\"pending_age\":0}" << std::endl;
        }
    }
}

} // namespace core
} // namespace vs_graphs
