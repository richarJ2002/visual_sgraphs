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
 * @file            findPassageSideRooms.cc
 *
 * @brief           Implements SemanticsManager::findPassageSideRooms(),
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

SemanticsManagerStatus SemanticsManager::findPassageSideRooms(
    const std::vector<semantic::Room *> &allRooms_in,
    semantic::Passage                   *p_passage_inout,
    PassageRoomLinkState                &linkState_inout)
{
    constexpr double sideEpsilon_m = 0.20;

    /* Track the closest room found on each side of the passage */
    linkState_inout.p_negativeSideRoom             = nullptr;
    linkState_inout.p_positiveSideRoom             = nullptr;
    linkState_inout.p_negativeExactSupportingOwner = nullptr;
    linkState_inout.p_positiveExactSupportingOwner = nullptr;

    linkState_inout.negativeRoomDistance_m = std::numeric_limits<double>::max();
    linkState_inout.positiveRoomDistance_m = std::numeric_limits<double>::max();

    /* Iterate through all valid rooms */
    for (vs_graphs::core::semantic::Room *p_room : allRooms_in)
    {
        /* Skip invalid rooms */
        bool roomIsBad2{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || roomIsBad2)
        {
            continue;
        }

        /* Extract the walls assigned to the room */
        std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /*
         * Match either the passage's source wall or a separately observed
         * wall surface at the same physical opening. Adjacent rooms must
         * not share one Plane pointer merely to obtain a graph edge.
         */
        bool hasSupportingWallGeometry = false;
        for (vs_graphs::core::geometric::Plane *p_roomWall : roomWalls)
        {
            bool isAtPassageOpening = false;
            if (isWallAtPassageOpening(p_roomWall,
                                       linkState_inout.supportingWalls,
                                       linkState_inout.passageEquation_world,
                                       linkState_inout.passageCentroid_world_m,
                                       linkState_inout.passageWidth_m,
                                       linkState_inout.openingFrameReach_m,
                                       isAtPassageOpening) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isWallAtPassageOpening returned a failure "
                             "status although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }
            if (isAtPassageOpening)
            {
                hasSupportingWallGeometry = true;
                break;
            }
        }

        /* A passage can only belong to a room when the wall it is linked
         * to (its supporting wall) belongs to that room. A room that owns
         * none of the passage's supporting wall surfaces - even one whose
         * free-space skeleton happens to cross the opening - must not be
         * connected to this passage. The consistency knot of the semantic
         * graph is the wall itself: the passage anchors to walls, and
         * walls anchor to exactly one room. */
        if (!hasSupportingWallGeometry)
        {
            continue;
        }

        bool ownsExactSupportingWall = false;
        for (vs_graphs::core::geometric::Plane *p_roomWall : roomWalls)
        {
            if (std::find(linkState_inout.supportingWalls.begin(),
                          linkState_inout.supportingWalls.end(),
                          p_roomWall) != linkState_inout.supportingWalls.end())
            {
                ownsExactSupportingWall = true;
                break;
            }
        }

        /* Association guard: exact ownership of the passage's supporting
         * wall is definitive adjacency evidence (handled above and
         * below). Anything weaker -- geometric proximity of some other
         * wall plus centroid distance -- may only compete for a side
         * when the room has enough boundary substance to make its
         * centroid meaningful. A single-wall room has no 2D extent; its
         * centroid sits on that one wall and wins whatever passage
         * happens to be nearest (typically right after a map reset,
         * latching the fresh room onto the wrong passage and locking it
         * in via wall ownership). Defer the edge until a second wall
         * arrives rather than invent topology. */
        if (!ownsExactSupportingWall)
        {
            constexpr std::size_t minimumWallsForProximityAssociation = 2U;
            std::size_t           validWallCount                      = 0U;
            for (geometric::Plane *p_roomWall : roomWalls)
            {
                bool roomWallIsBad{};
                if ((p_roomWall != nullptr) &&
                    p_roomWall->isBad(roomWallIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isBad returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_roomWall != nullptr && !roomWallIsBad)
                {
                    ++validWallCount;
                }
            }
            if (validWallCount < minimumWallsForProximityAssociation)
            {
                static std::set<std::pair<int, int>> reportedSparseSkips;
                int                                  passageId2{};
                if (p_passage_inout->getId(passageId2) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getId returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                int roomId{};
                if (p_room->getId(roomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getId returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (reportedSparseSkips.emplace(passageId2, roomId).second)
                {
                    int passageId3{};
                    if (p_passage_inout->getId(passageId3) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    int roomId2{};
                    if (p_room->getId(roomId2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    std::cout << "[SemMgr] Passage#" << passageId3
                              << " skipping Room#" << roomId2 << " (only "
                              << validWallCount << " valid wall(s); needs "
                              << minimumWallsForProximityAssociation
                              << " without exact supporting-wall ownership)."
                              << std::endl;
                }
                continue;
            }
        }

        /* Extract the room centroid */
        Eigen::Vector3d roomCentroid_world_m{};
        if (p_room->getCentroid(roomCentroid_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Determine which side of the passage plane contains the room */
        const double roomSide_m =
            linkState_inout.passageEquation_world.head<3>().dot(
                roomCentroid_world_m) +
            linkState_inout.passageEquation_world(3);

        /*!
         * A wall-centred provisional SE does not yet provide enough
         * evidence to form a room-to-passage connection -- UNLESS the
         * room owns the passage's own exact supporting wall. Owning
         * that wall is definitive, purely semantic (plane-equation)
         * evidence that the room borders this passage; which side of
         * the (nearly coincident, since the wall IS the passage's own
         * plane) passage plane the room's overall centroid happens to
         * land on is not meaningful evidence and must never veto it.
         * A sparsely-observed room (e.g. one confirmed wall so far)
         * can have its centroid sit within sideEpsilon_m purely
         * because that one known wall is this passage's supporting
         * wall -- silently dropping the room-to-passage edge every
         * cycle even though ownsExactSupportingWall already proves
         * the association.
         */
        if (!ownsExactSupportingWall && std::abs(roomSide_m) <= sideEpsilon_m)
        {
            continue;
        }

        /* Find the distance from the room to the passage */
        const double roomDistance_m =
            (roomCentroid_world_m - linkState_inout.passageCentroid_world_m)
                .norm();

        /* The centroid of a sparse room can lie on its only known wall.
         * Exact ownership of the passage's supporting wall is still
         * definitive adjacency evidence; use the stamped observation-side
         * provenance to break the otherwise-zero side test. */
        if (ownsExactSupportingWall && std::abs(roomSide_m) <= sideEpsilon_m)
        {
            bool knownSideHasDirection2{};
            if (linkState_inout.knownSide.hasDirection(
                    knownSideHasDirection2) !=
                semantic::KnownSideProvenanceStatus::
                    KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: hasDirection returned a failure status although "
                    "it cannot fail; continuing as before.",
                    __func__);
            }
            const double knownSideSign =
                knownSideHasDirection2
                    ? linkState_inout.knownSide.knownSideDirection_world.dot(
                          linkState_inout.passageEquation_world.head<3>())
                    : 0.0;
            if (knownSideSign >= 0.0 &&
                roomDistance_m < linkState_inout.positiveRoomDistance_m)
            {
                linkState_inout.positiveRoomDistance_m         = roomDistance_m;
                linkState_inout.p_positiveSideRoom             = p_room;
                linkState_inout.p_positiveExactSupportingOwner = p_room;
            }
            else if (knownSideSign < 0.0 &&
                     roomDistance_m < linkState_inout.negativeRoomDistance_m)
            {
                linkState_inout.negativeRoomDistance_m         = roomDistance_m;
                linkState_inout.p_negativeSideRoom             = p_room;
                linkState_inout.p_negativeExactSupportingOwner = p_room;
            }
            continue;
        }

        /* Keep the closest room on the negative side */
        if (roomSide_m < 0.0 &&
            roomDistance_m < linkState_inout.negativeRoomDistance_m)
        {
            linkState_inout.negativeRoomDistance_m = roomDistance_m;
            linkState_inout.p_negativeSideRoom     = p_room;
            linkState_inout.p_negativeExactSupportingOwner =
                ownsExactSupportingWall ? p_room : nullptr;
        }

        /* Keep the closest room on the positive side */
        if (roomSide_m > 0.0 &&
            roomDistance_m < linkState_inout.positiveRoomDistance_m)
        {
            linkState_inout.positiveRoomDistance_m = roomDistance_m;
            linkState_inout.p_positiveSideRoom     = p_room;
            linkState_inout.p_positiveExactSupportingOwner =
                ownsExactSupportingWall ? p_room : nullptr;
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
