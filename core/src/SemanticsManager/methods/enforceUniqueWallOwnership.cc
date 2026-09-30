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
 * @file            enforceUniqueWallOwnership.cc
 *
 * @brief           Implements SemanticsManager::enforceUniqueWallOwnership(),
 *                  declared in SemanticsManager.h.
 */

#include "SemanticsManager.h"

#include "../private_functions.h"
#include "Semantic/ValueOrder.h"

#include <algorithm>
#include <limits>
#include <rclcpp/logging.hpp>
#include <unordered_map>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::enforceUniqueWallOwnership(void)
{
    std::vector<vs_graphs::core::semantic::Room *> allRooms{};
    if (p_atlas->getAllRooms(allRooms) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::sort(allRooms.begin(),
              allRooms.end(),
              semantic::isEntityIdLess<semantic::Room>);

    std::vector<semantic::Passage *> allPassages{};
    if (p_atlas->getAllPassages(allPassages) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::sort(allPassages.begin(),
              allPassages.end(),
              semantic::isEntityIdLess<semantic::Passage>);

    Eigen::Vector3d   groundNormal_World = Eigen::Vector3d::Zero();
    geometric::Plane *p_groundPlane      = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool groundPlaneIsBad{};
    if ((p_groundPlane != nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlane != nullptr && !groundPlaneIsBad)
    {
        g2o::Plane3D groundPlaneGetGlobalEquation{};
        if (p_groundPlane->getGlobalEquation(groundPlaneGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d groundEquation =
            groundPlaneGetGlobalEquation.coeffs();
        const double groundNormalNorm = groundEquation.head<3>().norm();
        if (groundEquation.allFinite() && groundNormalNorm > 1e-8)
        {
            groundNormal_World = groundEquation.head<3>() / groundNormalNorm;
        }
    }

    std::unordered_map<geometric::Plane *, std::vector<semantic::Room *>>
        wallOwners;

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

            std::vector<semantic::Room *> &owners = wallOwners[p_wall];
            if (std::find(owners.begin(), owners.end(), p_room) == owners.end())
            {
                owners.push_back(p_room);
            }
        }
    }

    for (auto &[p_wall, owners] : wallOwners)
    {
        if (owners.size() < 2U)
        {
            continue;
        }

        semantic::Room                      *p_retainedOwner = nullptr;
        std::unordered_set<semantic::Room *> passageRejectedOwners;

        /* Passage-side routing is authoritative. A near-side owner whose
         * centroid-to-wall segment crosses an opening is not eligible; a live
         * stable far-side handle is preferred unless that would steal from a
         * different confirmed owner.
         *
         * Eligibility here is deliberately geometric only (isPassable(), the
         * passage's own detected-opening evidence) -- traversal evidence
         * (the camera/UAV having flown through this spot) proves only that
         * a room change happened there, not this passage's own aperture
         * geometry. Substituting it in as an OR-alternative would let a
         * geometrically-unconfirmed "passage" arbitrate which confirmed
         * room owns a contested wall, conflating motion evidence with wall
         * identity. */
        for (semantic::Room *p_nearOwner : owners)
        {
            for (semantic::Passage *p_passage : allPassages)
            {
                bool passageIsPassable{};
                if (!(p_passage == nullptr) &&
                    p_passage->isPassable(passageIsPassable) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isPassable returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector3d nearOwnerCentroid{};
                if (!(p_passage == nullptr || !passageIsPassable) &&
                    p_nearOwner->getCentroid(nearOwnerCentroid) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCentroid returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector3d wallGetCentroid{};
                if (!(p_passage == nullptr || !passageIsPassable) &&
                    p_wall->getCentroid(wallGetCentroid) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCentroid returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                bool crossesPassageOpening{};
                if (!(p_passage == nullptr || !passageIsPassable) &&
                    segmentCrossesPassageOpening(
                        nearOwnerCentroid,
                        wallGetCentroid.cast<double>(),
                        p_passage,
                        groundNormal_World,
                        p_sysParams->roomSeg.passagePartition.openingMargin_m,
                        p_sysParams->roomSeg.passagePartition
                            .minimumSideDistance_m,
                        crossesPassageOpening,
                        false) != SemanticsManagerStatus::
                                      SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: segmentCrossesPassageOpening returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_passage == nullptr || !passageIsPassable ||
                    !crossesPassageOpening)
                {
                    continue;
                }

                semantic::Room *p_farSideOwner = nullptr;
                if (p_passage->getProspectiveRoom(p_farSideOwner) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                bool farSideOwnerIsBad{};
                if (!(p_farSideOwner == nullptr) &&
                    p_farSideOwner->isBad(farSideOwnerIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                core::Map *p_farSideOwnerMap = nullptr;
                if (!(p_farSideOwner == nullptr || farSideOwnerIsBad ||
                      p_farSideOwner == p_nearOwner) &&
                    p_farSideOwner->getMap(p_farSideOwnerMap) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Map *p_atlasCurrentMap = nullptr;
                if (!(p_farSideOwner == nullptr || farSideOwnerIsBad ||
                      p_farSideOwner == p_nearOwner) &&
                    p_atlas->getCurrentMap(p_atlasCurrentMap) !=
                        AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCurrentMap returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_farSideOwner == nullptr || farSideOwnerIsBad ||
                    p_farSideOwner == p_nearOwner ||
                    p_farSideOwnerMap != p_atlasCurrentMap)
                {
                    passageRejectedOwners.insert(p_nearOwner);
                    continue;
                }

                const bool wouldStealDistinctConfirmedOwner = std::any_of(
                    owners.begin(),
                    owners.end(),
                    [p_nearOwner, p_farSideOwner](semantic::Room *p_owner)
                    {
                        semantic::Room::RoomVariant ownerRoomVariant{};
                        if ((p_owner != p_nearOwner &&
                             p_owner != p_farSideOwner) &&
                            p_owner->getRoomVariant(ownerRoomVariant) !=
                                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getRoomVariant returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        return p_owner != p_nearOwner &&
                               p_owner != p_farSideOwner &&
                               ownerRoomVariant ==
                                   semantic::Room::RoomVariant::ROOM;
                    });
                if (wouldStealDistinctConfirmedOwner)
                {
                    continue;
                }

                semantic::Room::RoomVariant farSideOwnerRoomVariant{};
                if (!(p_retainedOwner == nullptr) &&
                    p_farSideOwner->getRoomVariant(farSideOwnerRoomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRoomVariant returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                semantic::Room::RoomVariant retainedOwnerRoomVariant{};
                if (!(p_retainedOwner == nullptr) &&
                    (farSideOwnerRoomVariant ==
                     semantic::Room::RoomVariant::ROOM) &&
                    p_retainedOwner->getRoomVariant(retainedOwnerRoomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRoomVariant returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                semantic::Room::RoomVariant farSideOwnerRoomVariant2{};
                if (!(p_retainedOwner == nullptr ||
                      (farSideOwnerRoomVariant ==
                           semantic::Room::RoomVariant::ROOM &&
                       retainedOwnerRoomVariant !=
                           semantic::Room::RoomVariant::ROOM)) &&
                    p_farSideOwner->getRoomVariant(farSideOwnerRoomVariant2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRoomVariant returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                semantic::Room::RoomVariant retainedOwnerRoomVariant2{};
                if (!(p_retainedOwner == nullptr ||
                      (farSideOwnerRoomVariant ==
                           semantic::Room::RoomVariant::ROOM &&
                       retainedOwnerRoomVariant !=
                           semantic::Room::RoomVariant::ROOM)) &&
                    p_retainedOwner->getRoomVariant(
                        retainedOwnerRoomVariant2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRoomVariant returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                int farSideOwnerId{};
                if (!(p_retainedOwner == nullptr ||
                      (farSideOwnerRoomVariant ==
                           semantic::Room::RoomVariant::ROOM &&
                       retainedOwnerRoomVariant !=
                           semantic::Room::RoomVariant::ROOM)) &&
                    (farSideOwnerRoomVariant2 == retainedOwnerRoomVariant2) &&
                    p_farSideOwner->getId(farSideOwnerId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int retainedOwnerId{};
                if (!(p_retainedOwner == nullptr ||
                      (farSideOwnerRoomVariant ==
                           semantic::Room::RoomVariant::ROOM &&
                       retainedOwnerRoomVariant !=
                           semantic::Room::RoomVariant::ROOM)) &&
                    (farSideOwnerRoomVariant2 == retainedOwnerRoomVariant2) &&
                    p_retainedOwner->getId(retainedOwnerId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_retainedOwner == nullptr ||
                    (farSideOwnerRoomVariant ==
                         semantic::Room::RoomVariant::ROOM &&
                     retainedOwnerRoomVariant !=
                         semantic::Room::RoomVariant::ROOM) ||
                    (farSideOwnerRoomVariant2 == retainedOwnerRoomVariant2 &&
                     farSideOwnerId < retainedOwnerId))
                {
                    p_retainedOwner = p_farSideOwner;
                }
            }
        }

        /* Existing confirmed ownership outranks camera proximity. */
        if (p_retainedOwner == nullptr)
        {
            for (semantic::Room *p_owner : owners)
            {
                semantic::Room::RoomVariant ownerRoomVariant{};
                if ((passageRejectedOwners.count(p_owner) == 0U) &&
                    p_owner->getRoomVariant(ownerRoomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRoomVariant returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (passageRejectedOwners.count(p_owner) == 0U &&
                    ownerRoomVariant == semantic::Room::RoomVariant::ROOM)
                {
                    p_retainedOwner = p_owner;
                    break;
                }
            }
        }

        /* Camera proximity is the final fallback among equivalent/provisional
         * owners only. */
        if (p_retainedOwner == nullptr)
        {
            Eigen::Vector3d meanObservationPosition_World_m =
                Eigen::Vector3d::Zero();
            std::size_t validObservationCount = 0U;

            std::map<core::KeyFrame *, geometric::Plane::Observation>
                wallGetObservations{};
            if (p_wall->getObservations(wallGetObservations) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getObservations returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (const auto &[p_keyFrame, observation] : wallGetObservations)
            {
                static_cast<void>(observation);

                bool keyFrameIsBad{};
                if ((p_keyFrame != nullptr) &&
                    p_keyFrame->isBad(keyFrameIsBad) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_keyFrame != nullptr && !keyFrameIsBad)
                {
                    Eigen::Vector3f keyFrameCameraCenter{};
                    if (p_keyFrame->getCameraCenter(keyFrameCameraCenter) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCameraCenter returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    const Eigen::Vector3d cameraCenter_World_m =
                        keyFrameCameraCenter.cast<double>();

                    if (cameraCenter_World_m.allFinite())
                    {
                        meanObservationPosition_World_m += cameraCenter_World_m;
                        validObservationCount++;
                    }
                }
            }

            if (validObservationCount > 0U)
            {
                meanObservationPosition_World_m /=
                    static_cast<double>(validObservationCount);
                double bestDistance_m = std::numeric_limits<double>::infinity();

                for (semantic::Room *p_owner : owners)
                {
                    if (passageRejectedOwners.count(p_owner) > 0U)
                    {
                        continue;
                    }

                    Eigen::Vector3d ownerCentroid{};
                    if (p_owner->getCentroid(ownerCentroid) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getCentroid returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    const double distance_m =
                        (ownerCentroid - meanObservationPosition_World_m)
                            .norm();
                    if (distance_m < bestDistance_m)
                    {
                        bestDistance_m  = distance_m;
                        p_retainedOwner = p_owner;
                    }
                }
            }

            if (p_retainedOwner == nullptr)
            {
                for (semantic::Room *p_owner : owners)
                {
                    if (passageRejectedOwners.count(p_owner) == 0U)
                    {
                        p_retainedOwner = p_owner;
                        break;
                    }
                }
            }
        }

        for (semantic::Room *p_owner : owners)
        {
            bool ownerWasWallRemoved{};
            if ((p_owner != p_retainedOwner) &&
                p_owner->removeWall(p_wall, ownerWasWallRemoved) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                ownerWasWallRemoved = false;
                RCLCPP_WARN(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: removeWall rejected its input; continuing as before.",
                    __func__);
            }
            if (p_owner != p_retainedOwner && ownerWasWallRemoved)
            {
                int retainedOwnerId2{};
                if ((p_retainedOwner != nullptr) &&
                    p_retainedOwner->getId(retainedOwnerId2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int ownerId{};
                if (p_owner->getId(ownerId) !=
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
                std::cerr << "[SemMgr] Corrected duplicate ownership of Wall#"
                          << wallGetId << ": "
                          << (p_retainedOwner != nullptr
                                  ? "retained semantic::Room#" +
                                        std::to_string(retainedOwnerId2)
                                  : "left orphaned")
                          << ", detached semantic::Room#" << ownerId << "."
                          << std::endl;
            }
        }

        if (p_retainedOwner != nullptr &&
            std::find(owners.begin(), owners.end(), p_retainedOwner) ==
                owners.end())
        {
            if (p_retainedOwner->setWalls(p_wall) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
