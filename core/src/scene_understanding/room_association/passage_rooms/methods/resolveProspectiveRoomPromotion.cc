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
 * @file            resolveProspectiveRoomPromotion.cc
 *
 * @brief           Implements
 * SemanticsManager::resolveProspectiveRoomPromotion(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
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

SemanticsManagerStatus SemanticsManager::resolveProspectiveRoomPromotion(
    const std::vector<semantic::Room *> &allRooms_in,
    semantic::Passage                   *p_passage_inout)
{
    /* ----------------------------------------------------------------------
     * * PROSPECTIVE ROOM PROMOTION
     * ----------------------------------------------------------------------
     * * Wall ownership alone cannot promote a prospective. Promotion is
     * performed only by associateWallsToRoomsFromFreeSpaceClusters() after
     * an independent far-side cluster matches the prospective's existing
     * walls. If a distinct confirmed room already resolves the far side,
     * retire the placeholder and preserve that room as the passage's
     * stable handle.
     */
    bool passageHasProspectiveRoom4{};
    if (p_passage_inout->hasProspectiveRoom(passageHasProspectiveRoom4) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasProspectiveRoom returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (passageHasProspectiveRoom4)
    {
        vs_graphs::core::semantic::Room *p_prospectiveRoom = nullptr;
        if (p_passage_inout->getProspectiveRoom(p_prospectiveRoom) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        bool prospectiveRoomIsBad{};
        if ((p_prospectiveRoom != nullptr) &&
            p_prospectiveRoom->isBad(prospectiveRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room::RoomVariant prospectiveRoomRoomVariant{};
        if ((p_prospectiveRoom != nullptr && !prospectiveRoomIsBad) &&
            p_prospectiveRoom->getRoomVariant(prospectiveRoomRoomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<geometric::Plane *> prospectiveRoomWalls{};
        if ((p_prospectiveRoom != nullptr && !prospectiveRoomIsBad &&
             prospectiveRoomRoomVariant ==
                 vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED) &&
            p_prospectiveRoom->getWalls(prospectiveRoomWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_prospectiveRoom != nullptr && !prospectiveRoomIsBad &&
            prospectiveRoomRoomVariant ==
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED &&
            !prospectiveRoomWalls.empty())
        {
            /* Find a confirmed (non-prospective) room on the FAR side of
             * the passage - i.e. on the same side as the prospective room.
             * Use ANY such room, not just the strictly associated side
             * rooms, so that a passage opening onto a corridor or another
             * already-mapped room stops being prospective as soon as that
             * room exists. When one is found, the prospective placeholder
             * is deleted - it must not linger as a loose provisional room.
             */
            vs_graphs::core::semantic::Room *p_farSideConfirmedRoom = nullptr;

            /*!
             * Driving principle of far-side ownership: "a wall observed on
             * the far side of ANY confirmed passage aperture belongs to
             * that passage's prospective room... decide which side of every
             * passage it lies on" -- using segmentCrossesPassageOpening,
             * the same purely semantic (passage width/height aperture,
             * not voxblox cluster geometry) test already used for wall
             * admission (e.g. line ~3759). A same-side-of-the-infinite-
             * plane sign test is NOT proof of adjacency: a room several
             * metres past this passage (only reachable through an
             * intervening, not-yet-confirmed room) satisfies "same
             * side" just as well as a genuinely bordering room does.
             * Observed directly: Passage#1 sitting between Room#4 and a
             * distant Room#5 kept resolving straight to Room#5,
             * destroying the middle prospective room meant to sit
             * between them, every cycle. Testing whether the segment
             * between the two room centroids actually threads through
             * THIS passage's own bounded opening (not just crosses its
             * infinite plane somewhere) rejects that distant match
             * without any distance threshold borrowed from an unrelated
             * (voxblox free-space cluster) subsystem.
             *
             * This still isn't sufficient on its own when the
             * prospective room is nothing but its creation-time
             * heuristic position (passage_centroid + normal * an
             * assumed depth, before any real wall has been admitted to
             * it): in a straight corridor with several doors in a row,
             * that guessed point and a genuinely distant, unrelated
             * room can both sit close enough to the corridor centreline
             * for the straight segment between them to thread THIS
             * passage's aperture too, even though a different room and
             * passage lie directly between them. Observed directly:
             * a freshly created prospective room, still with zero
             * walls, resolved straight to a confirmed room three doors
             * down the same corridor on the very cycle it was created.
             * Requiring at least one real, admitted wall first (the
             * same evidence bar promotion already applies below: "wall
             * ownership alone cannot promote... validated far-side
             * cluster evidence may promote") anchors the near endpoint
             * of the crossing test to an actually observed position
             * instead of an unvalidated depth guess.
             *
             * Also excludes the passage's own near-side/known room from
             * candidacy, mirroring the creation-time anti-churn guard
             * above (§7342-7374) -- a room that owns the passage's own
             * near side must never be matched as its far side.
             */
            if (findPromotionFarSideRoom(allRooms_in,
                                         p_passage_inout,
                                         p_prospectiveRoom,
                                         p_farSideConfirmedRoom) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: findPromotionFarSideRoom returned a failure "
                             "status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }

            if (p_farSideConfirmedRoom != nullptr)
            {
                if (retireProspectiveRoom(p_passage_inout,
                                          p_prospectiveRoom,
                                          p_farSideConfirmedRoom) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: retireProspectiveRoom returned a failure "
                                 "status although it "
                                 "cannot fail; continuing as before.",
                                 __func__);
                }
            }
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
