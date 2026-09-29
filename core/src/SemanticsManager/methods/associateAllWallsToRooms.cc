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
#include <rclcpp/logging.hpp>

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
        bool roomIsBad{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        core::Map *p_roomMap = nullptr;
        if ((p_room != nullptr && !roomIsBad) &&
            p_room->getMap(p_roomMap) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int roomId{};
        if ((p_room != nullptr && !roomIsBad && p_roomMap == p_activeMap) &&
            p_room->getId(roomId) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room::RoomVariant roomVariant{};
        if ((p_room != nullptr && !roomIsBad && p_roomMap == p_activeMap &&
             roomId == currentRoomIdSnapshot) &&
            p_room->getRoomVariant(roomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad && p_roomMap == p_activeMap &&
            roomId == currentRoomIdSnapshot &&
            roomVariant == semantic::Room::RoomVariant::ROOM)
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
    bool            groundPlaneForEvidenceIsBad{};
    if ((p_groundPlaneForEvidence != nullptr) &&
        p_groundPlaneForEvidence->isBad(groundPlaneForEvidenceIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlaneForEvidence != nullptr && !groundPlaneForEvidenceIsBad)
    {
        g2o::Plane3D groundPlaneForEvidenceGetGlobalEquation{};
        if (p_groundPlaneForEvidence->getGlobalEquation(
                groundPlaneForEvidenceGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d groundEquation =
            groundPlaneForEvidenceGetGlobalEquation.coeffs();
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
        std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
        if (room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Check whether the requested wall is already present */
        return std::any_of(
            roomWalls.begin(),
            roomWalls.end(),
            [p_wall](vs_graphs::core::geometric::Plane *p_existingWall)
            {
                int existingWallGetId{};
                if ((p_existingWall != nullptr) &&
                    p_existingWall->getId(existingWallGetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int wallGetId{};
                if ((p_existingWall != nullptr) &&
                    p_wall->getId(wallGetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                return p_existingWall != nullptr &&
                       existingWallGetId == wallGetId;
            });
    };

    /* Iterate through every mapped plane */
    for (vs_graphs::core::geometric::Plane *p_wall : allPlanes)
    {
        /* Skip invalid planes */
        bool wallIsBad{};
        if (!(p_wall == nullptr) &&
            p_wall->isBad(wallIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_wall == nullptr || wallIsBad)
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
            geometric::Plane::PlaneVariant wallPlaneType{};
            if (p_wall->getPlaneType(wallPlaneType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPlaneType returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            geometric::Plane::PlaneVariant wallExpectedPlaneType{};
            if (!(wallPlaneType != geometric::Plane::PlaneVariant::WALL) &&
                p_wall->getExpectedPlaneType(wallExpectedPlaneType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getExpectedPlaneType returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            const std::string reason =
                wallPlaneType != geometric::Plane::PlaneVariant::WALL ||
                        wallExpectedPlaneType !=
                            geometric::Plane::PlaneVariant::WALL
                    ? "CLASS_NOT_WALL"
                : !admissionEvidence.hasAdequateFiniteFit
                    ? "INADEQUATE_FINITE_FIT"
                    : "INSUFFICIENT_OBSERVATIONS";
            int wallGetId{};
            if (p_wall->getId(wallGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (loggedWallRejectionReasons[wallGetId] != reason)
            {
                int wallGetId2{};
                if (p_wall->getId(wallGetId2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                loggedWallRejectionReasons[wallGetId2] = reason;
                int wallGetId3{};
                if (p_wall->getId(wallGetId3) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                geometric::Plane::PlaneVariant wallPlaneType2{};
                if (p_wall->getPlaneType(wallPlaneType2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPlaneType returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                std::cout << "SG_PIPELINE {\"event\":\"wall_rejection\","
                             "\"map_id\":"
                          << (p_activeMap != nullptr
                                  ? static_cast<long long>(p_activeMap->getId())
                                  : -1)
                          << ",\"semantic_cycle\":" << pipelineSemanticCycle
                          << ",\"wall_id\":" << wallGetId3 << ",\"class\":\""
                          << planeClassName(wallPlaneType2)
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
        int wallGetId4{};
        if (p_wall->getId(wallGetId4) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        loggedWallRejectionReasons.erase(wallGetId4);

        /* Init flag which confirms whether the wall already has a parent */
        bool wallHasRoom = false;

        /* Search every valid room for the wall */
        for (vs_graphs::core::semantic::Room *room : allRooms)
        {
            /* Skip invalid rooms */
            bool roomIsBad2{};
            if (!(room == nullptr) &&
                room->isBad(roomIsBad2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (room == nullptr || roomIsBad2)
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
            bool roomIsBad3{};
            if ((p_room != nullptr) &&
                p_room->isBad(roomIsBad3) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room != nullptr && !roomIsBad3 &&
                roomContainsWall(p_room, p_wall))
            {
                p_selectedOwner = p_room;
                break;
            }
        }

        if (admitted && p_selectedOwner != nullptr)
        {
            int wallGetId5{};
            if (p_wall->getId(wallGetId5) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_atlas->getRoomWallPlaneById(wallGetId5) == nullptr)
            {
                p_atlas->addRoomWallPlane(p_wall);
            }
            int wallGetId6{};
            if (p_wall->getId(wallGetId6) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            undefendedWalls.erase(wallGetId6);
            int wallGetId7{};
            if (p_wall->getId(wallGetId7) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            loggedOrphanWallIds.erase(wallGetId7);
            int selectedOwnerId{};
            if (p_selectedOwner->getId(selectedOwnerId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int wallGetId8{};
            if (p_wall->getId(wallGetId8) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "SG_PIPELINE {\"event\":\"wall_admission\","
                         "\"map_id\":"
                      << p_activeMap->getId()
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle
                      << ",\"wall_id\":" << wallGetId8
                      << ",\"class\":\"WALL\","
                         "\"lifecycle\":\"COMMITTED\",\"owner_room_id\":"
                      << selectedOwnerId << ",\"reason\":\""
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
        int wallGetId9{};
        if (p_wall->getId(wallGetId9) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_atlas->getRoomWallPlaneById(wallGetId9) == nullptr)
        {
            p_atlas->addRoomWallPlane(p_wall);
        }

        int wallGetId10{};
        if (p_wall->getId(wallGetId10) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (loggedOrphanWallIds.insert(wallGetId10).second)
        {
            int wallGetId11{};
            if (p_wall->getId(wallGetId11) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "SG_PIPELINE {\"event\":\"wall_pending\","
                         "\"map_id\":"
                      << (p_activeMap != nullptr
                              ? static_cast<long long>(p_activeMap->getId())
                              : -1)
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle
                      << ",\"wall_id\":" << wallGetId11
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
