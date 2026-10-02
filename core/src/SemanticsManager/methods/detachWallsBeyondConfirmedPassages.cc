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
 * @file            detachWallsBeyondConfirmedPassages.cc
 *
 * @brief           Implements
 *                  SemanticsManager::detachWallsBeyondConfirmedPassages(),
 *                  declared in SemanticsManager.h.
 */

#include "SemanticsManager.h"

#include "../private_functions.h"
#include "Semantic/ValueOrder.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::detachWallsBeyondConfirmedPassages(void)
{
    const types::SystemParams::RoomSeg::PassagePartition &partitionParameters =
        p_sysParams->roomSeg.passagePartition;

    if (!partitionParameters.enabled ||
        !partitionParameters.shouldDetachWallsBeyondPassages)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    geometric::Plane *p_groundPlane = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    bool groundPlaneIsBad{};
    if (!(p_groundPlane == nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlane == nullptr || groundPlaneIsBad)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    g2o::Plane3D groundPlaneGetGlobalEquation{};
    if (p_groundPlane->getGlobalEquation(groundPlaneGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector4d groundEquation_world =
        groundPlaneGetGlobalEquation.coeffs();
    const double groundNormalNorm = groundEquation_world.head<3>().norm();

    if (!groundEquation_world.allFinite() || groundNormalNorm < 1e-8)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const Eigen::Vector3d groundNormal_world =
        groundEquation_world.head<3>() / groundNormalNorm;
    const double openingMargin_m =
        static_cast<double>(partitionParameters.openingMargin_m);
    const double minimumSideDistance_m = static_cast<double>(
        partitionParameters.wallCentroidMinimumSideDistance_m);

    std::vector<semantic::Passage *> confirmedOpenPassages;

    std::vector<vs_graphs::core::semantic::Passage *> atlasAllPassages{};
    if (p_atlas->getAllPassages(atlasAllPassages) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (semantic::Passage *p_passage : atlasAllPassages)
    {
        bool passageIsPassable{};
        if ((p_passage != nullptr) &&
            p_passage->isPassable(passageIsPassable) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isPassable returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_passage != nullptr && passageIsPassable)
        {
            confirmedOpenPassages.push_back(p_passage);
        }
    }

    std::sort(confirmedOpenPassages.begin(),
              confirmedOpenPassages.end(),
              semantic::isEntityIdLess<semantic::Passage>);

    if (confirmedOpenPassages.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::vector<semantic::Room *> allRooms{};
    if (p_atlas->getAllRooms(allRooms) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    for (semantic::Room *p_room : allRooms)
    {
        bool roomIsBad{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || roomIsBad)
        {
            continue;
        }

        Eigen::Vector3d roomCentroid_world_m{};
        if (p_room->getCentroid(roomCentroid_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (!roomCentroid_world_m.allFinite())
        {
            continue;
        }

        std::vector<geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_wall : roomWalls)
        {
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

            Eigen::Vector3d wallGetCentroid{};
            if (p_wall->getCentroid(wallGetCentroid) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const Eigen::Vector3d wallCentroid_world_m =
                wallGetCentroid.cast<double>();

            if (!wallCentroid_world_m.allFinite())
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
                bool crossesPassageOpening{};
                if (segmentCrossesPassageOpening(roomCentroid_world_m,
                                                 wallCentroid_world_m,
                                                 p_passage,
                                                 groundNormal_world,
                                                 openingMargin_m,
                                                 minimumSideDistance_m,
                                                 crossesPassageOpening) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: segmentCrossesPassageOpening returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                if (crossesPassageOpening)
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
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                semantic::Room::RoomVariant otherRoomRoomVariant{};
                if (!(p_otherRoom == nullptr || p_otherRoom == p_room ||
                      otherRoomIsBad) &&
                    p_otherRoom->getRoomVariant(otherRoomRoomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRoomVariant returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
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
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWalls returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
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
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            bool farSideRoomIsBad{};
            if ((p_farSideRoom != nullptr) &&
                p_farSideRoom->isBad(farSideRoomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
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
                roomWasWallRemoved = false;
                RCLCPP_WARN(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: removeWall rejected its input; continuing as before.",
                    __func__);
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
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int confirmedOwnerId{};
                if (p_confirmedOwner->getId(confirmedOwnerId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int wallGetId{};
                if (p_wall->getId(wallGetId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout
                    << "[SemMgr] Detached far-side Wall#" << wallGetId
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
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setWalls returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                int wallGetId2{};
                if (p_wall->getId(wallGetId2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                vs_graphs::core::geometric::Plane *p_atlasRoomWallPlaneById =
                    nullptr;
                if (p_atlas->getRoomWallPlaneById(wallGetId2,
                                                  p_atlasRoomWallPlaneById) !=
                    AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRoomWallPlaneById returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_atlasRoomWallPlaneById == nullptr)
                {
                    if (p_atlas->addRoomWallPlane(p_wall) !=
                        AtlasStatus::ATLAS_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addRoomWallPlane returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                }
                int roomId2{};
                if (p_room->getId(roomId2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int separatingPassageId{};
                if (p_separatingPassage->getId(separatingPassageId) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int farSideRoomId{};
                if (p_farSideRoom->getId(farSideRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int wallGetId3{};
                if (p_wall->getId(wallGetId3) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "[SemMgr] Redirected far-side Wall#" << wallGetId3
                          << " from semantic::Room#" << roomId2
                          << " through semantic::Passage#"
                          << separatingPassageId << " to stable semantic::Room#"
                          << farSideRoomId << "." << std::endl;
                continue;
            }

            int roomId3{};
            if (p_room->getId(roomId3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int separatingPassageId2{};
            if (p_separatingPassage->getId(separatingPassageId2) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
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
            std::cout << "[SemMgr] Detached far-side Wall#" << wallGetId4
                      << " from semantic::Room#" << roomId3
                      << "; semantic::Passage#" << separatingPassageId2
                      << " has no stable far-side room, so the wall remains "
                         "orphaned."
                      << std::endl;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
