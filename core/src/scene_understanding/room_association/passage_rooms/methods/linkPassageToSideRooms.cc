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
 * @file            linkPassageToSideRooms.cc
 *
 * @brief           Implements SemanticsManager::linkPassageToSideRooms(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "configuration/types/types.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/geo_sem_helpers/objects/GeoSemHelpersStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::linkPassageToSideRooms(
    const std::vector<semantic::Room *> &allRooms_in,
    const std::unordered_map<semantic::Room *, std::unordered_set<int>>
                         &previousPassageIdsByRoom_in,
    semantic::Passage    *p_passage_inout,
    PassageRoomLinkState &linkState_inout)
{

    /*!
     * Preserve every geometrically supported room-to-passage edge.
     *
     * A free-space ray may confirm an opening before the room on the far
     * side has enough boundary evidence to exist in the semantic graph.
     * Retaining the known-side edge represents that passage as a frontier
     * without inventing a second room. Once distinct rooms are observed on
     * both sides, the same two edges form the complete room-to-room route.
     */
    if (linkRoomToPassage(linkState_inout.p_negativeSideRoom,
                          p_passage_inout,
                          previousPassageIdsByRoom_in) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: linkRoomToPassage returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }

    if (linkState_inout.p_positiveSideRoom !=
        linkState_inout.p_negativeSideRoom)
    {
        if (linkRoomToPassage(linkState_inout.p_positiveSideRoom,
                              p_passage_inout,
                              previousPassageIdsByRoom_in) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: linkRoomToPassage returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
    }

    /* Enforce the invariant that a passage is linked to AT MOST TWO rooms
     * (the nearest room on each side of its supporting wall). Rooms that
     * no longer win their side - or that are duplicate hypotheses of the
     * winning side - must have this passage association revoked so the
     * semantic graph never shows a passage with three rooms. */
    for (vs_graphs::core::semantic::Room *p_candidateRoom : allRooms_in)
    {
        bool candidateRoomIsBad{};
        if (!(p_candidateRoom == nullptr) &&
            p_candidateRoom->isBad(candidateRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_candidateRoom == nullptr || candidateRoomIsBad ||
            p_candidateRoom == linkState_inout.p_negativeSideRoom ||
            p_candidateRoom == linkState_inout.p_positiveSideRoom)
        {
            continue;
        }

        bool candidateRoomWasPassageRemoved{};
        if (p_candidateRoom->removePassageAssociation(
                p_passage_inout,
                candidateRoomWasPassageRemoved) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            candidateRoomWasPassageRemoved = false;
            RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                        "%s: removePassageAssociation rejected its input; "
                        "continuing as before.",
                        __func__);
        }
        if (candidateRoomWasPassageRemoved)
        {
            int passageId5{};
            if (p_passage_inout->getId(passageId5) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            int candidateRoomId{};
            if (p_candidateRoom->getId(candidateRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Revoked Passage#" << passageId5
                      << " from Room#" << candidateRoomId
                      << " (enforcing max-2-rooms-per-passage)." << std::endl;
        }
    }

    /* Enforce 1-2 room invariant for this passage */
    linkState_inout.associatedRoomCount          = 0;
    linkState_inout.confirmedAssociatedRoomCount = 0;
    linkState_inout.p_confirmedAssociatedRoom    = nullptr;
    linkState_inout.p_undefinedAssociatedRoom    = nullptr;

    /* The negative side counts first, then a distinct positive side. */
    const std::array<vs_graphs::core::semantic::Room *, 2U> sideRooms = {
        linkState_inout.p_negativeSideRoom,
        linkState_inout.p_positiveSideRoom != linkState_inout.p_negativeSideRoom
            ? linkState_inout.p_positiveSideRoom
            : nullptr};
    for (vs_graphs::core::semantic::Room *p_sideRoom : sideRooms)
    {
        if (p_sideRoom == nullptr)
        {
            continue;
        }
        linkState_inout.associatedRoomCount++;

        semantic::Room::RoomVariant roomVariant{};
        if (p_sideRoom->getRoomVariant(roomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (roomVariant ==
            vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            linkState_inout.p_undefinedAssociatedRoom = p_sideRoom;
            continue;
        }

        linkState_inout.confirmedAssociatedRoomCount++;
        linkState_inout.p_confirmedAssociatedRoom = p_sideRoom;
    }

    bool knownSideHasDirection3{};
    if (linkState_inout.knownSide.hasDirection(knownSideHasDirection3) !=
        semantic::KnownSideProvenanceStatus::
            KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    linkState_inout.knownSideSign =
        knownSideHasDirection3
            ? linkState_inout.knownSide.knownSideDirection_world.dot(
                  linkState_inout.passageEquation_world.head<3>())
            : 0.0;

    if (linkState_inout.knownSide.p_room == nullptr)
    {
        bool isNegativeRoomOnKnownSide = false;
        if (isRoomOnKnownSide(linkState_inout.p_negativeSideRoom,
                              linkState_inout.passageEquation_world,
                              linkState_inout.knownSide,
                              linkState_inout.knownSideSign,
                              isNegativeRoomOnKnownSide) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: isRoomOnKnownSide returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        bool isPositiveRoomOnKnownSide = false;
        if (isRoomOnKnownSide(linkState_inout.p_positiveSideRoom,
                              linkState_inout.passageEquation_world,
                              linkState_inout.knownSide,
                              linkState_inout.knownSideSign,
                              isPositiveRoomOnKnownSide) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: isRoomOnKnownSide returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        semantic::Room *p_knownSideRoom =
            isNegativeRoomOnKnownSide
                ? linkState_inout.p_negativeSideRoom
                : (isPositiveRoomOnKnownSide
                       ? linkState_inout.p_positiveSideRoom
                       : nullptr);
        if (p_knownSideRoom != nullptr)
        {
            if (p_passage_inout->setKnownSideRoom(p_knownSideRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setKnownSideRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Passage::KnownSideProvenance
                passageKnownSideProvenance2{};
            if (p_passage_inout->getKnownSideProvenance(
                    passageKnownSideProvenance2) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getKnownSideProvenance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            linkState_inout.knownSide = passageKnownSideProvenance2;
        }
    }

    if (linkState_inout.associatedRoomCount > 2)
    {
        int passageId11{};
        if (p_passage_inout->getId(passageId11) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemMgr] WARNING: Passage#" << passageId11 << " has "
                  << linkState_inout.associatedRoomCount
                  << " associated rooms; expected max 2." << std::endl;
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
