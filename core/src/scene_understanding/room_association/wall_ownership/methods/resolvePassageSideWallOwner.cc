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
 * @file            resolvePassageSideWallOwner.cc
 *
 * @brief           Implements SemanticsManager::resolvePassageSideWallOwner(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <limits>
#include <rclcpp/logging.hpp>
#include <unordered_map>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::resolvePassageSideWallOwner(
    geometric::Plane                       *p_wall_in,
    const std::vector<semantic::Room *>    &owners_in,
    const std::vector<semantic::Passage *> &allPassages_in,
    const Eigen::Vector3d                  &groundNormal_world_in,
    semantic::Room                        *&p_retainedOwner_out,
    std::unordered_set<semantic::Room *>   &passageRejectedOwners_out)
{
    p_retainedOwner_out = nullptr;
    passageRejectedOwners_out.clear();

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
    for (semantic::Room *p_nearOwner : owners_in)
    {
        for (semantic::Passage *p_passage : allPassages_in)
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
                p_wall_in->getCentroid(wallGetCentroid) !=
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
                    groundNormal_world_in,
                    p_sysParams->roomSeg.passagePartition.openingMargin_m,
                    p_sysParams->roomSeg.passagePartition.minimumSideDistance_m,
                    crossesPassageOpening,
                    false) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
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
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
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
            map::Map *p_farSideOwnerMap = nullptr;
            if (!(p_farSideOwner == nullptr || farSideOwnerIsBad ||
                  p_farSideOwner == p_nearOwner) &&
                p_farSideOwner->getMap(p_farSideOwnerMap) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            map::Map *p_atlasCurrentMap = nullptr;
            if (!(p_farSideOwner == nullptr || farSideOwnerIsBad ||
                  p_farSideOwner == p_nearOwner) &&
                p_atlas->getCurrentMap(p_atlasCurrentMap) !=
                    atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
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
                passageRejectedOwners_out.insert(p_nearOwner);
                continue;
            }

            bool wouldStealDistinctConfirmedOwner = false;
            for (semantic::Room *p_owner : owners_in)
            {
                semantic::Room::RoomVariant ownerRoomVariant{};
                if ((p_owner != p_nearOwner && p_owner != p_farSideOwner) &&
                    p_owner->getRoomVariant(ownerRoomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getRoomVariant returned a "
                                 "failure status although it cannot "
                                 "fail; continuing as before.",
                                 __func__);
                }
                if (p_owner != p_nearOwner && p_owner != p_farSideOwner &&
                    ownerRoomVariant == semantic::Room::RoomVariant::ROOM)
                {
                    wouldStealDistinctConfirmedOwner = true;
                    break;
                }
            }
            if (wouldStealDistinctConfirmedOwner)
            {
                continue;
            }

            semantic::Room::RoomVariant farSideOwnerRoomVariant{};
            if (!(p_retainedOwner_out == nullptr) &&
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
            if (!(p_retainedOwner_out == nullptr) &&
                (farSideOwnerRoomVariant ==
                 semantic::Room::RoomVariant::ROOM) &&
                p_retainedOwner_out->getRoomVariant(retainedOwnerRoomVariant) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getRoomVariant returned a failure status although "
                    "it cannot fail; continuing as before.",
                    __func__);
            }
            semantic::Room::RoomVariant farSideOwnerRoomVariant2{};
            if (!(p_retainedOwner_out == nullptr ||
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
            if (!(p_retainedOwner_out == nullptr ||
                  (farSideOwnerRoomVariant ==
                       semantic::Room::RoomVariant::ROOM &&
                   retainedOwnerRoomVariant !=
                       semantic::Room::RoomVariant::ROOM)) &&
                p_retainedOwner_out->getRoomVariant(
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
            if (!(p_retainedOwner_out == nullptr ||
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
            if (!(p_retainedOwner_out == nullptr ||
                  (farSideOwnerRoomVariant ==
                       semantic::Room::RoomVariant::ROOM &&
                   retainedOwnerRoomVariant !=
                       semantic::Room::RoomVariant::ROOM)) &&
                (farSideOwnerRoomVariant2 == retainedOwnerRoomVariant2) &&
                p_retainedOwner_out->getId(retainedOwnerId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_retainedOwner_out == nullptr ||
                (farSideOwnerRoomVariant == semantic::Room::RoomVariant::ROOM &&
                 retainedOwnerRoomVariant !=
                     semantic::Room::RoomVariant::ROOM) ||
                (farSideOwnerRoomVariant2 == retainedOwnerRoomVariant2 &&
                 farSideOwnerId < retainedOwnerId))
            {
                p_retainedOwner_out = p_farSideOwner;
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
