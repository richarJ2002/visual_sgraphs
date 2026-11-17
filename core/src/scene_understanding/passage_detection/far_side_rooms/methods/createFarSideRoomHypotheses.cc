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
 * @file            createFarSideRoomHypotheses.cc
 *
 * @brief           Implements SemanticsManager::createFarSideRoomHypotheses(),
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

SemanticsManagerStatus SemanticsManager::createFarSideRoomHypotheses(
    const std::vector<semantic::Room *>    &allRooms_in,
    const std::vector<semantic::Passage *> &allPassages_in,
    semantic::Passage                      *p_passage_inout,
    PassageRoomLinkState                   &linkState_inout)
{
    /* ----------------------------------------------------------------------
     * * PROSPECTIVE ROOM CREATION
     * ----------------------------------------------------------------------
     * * When a passage has exactly 1 associated room, create a PROVISIONAL
     * (UNDEFINED variant) room on the far side. This represents the spatial
     * hypothesis that traversable space continues beyond the opening.
     *
     * The prospective room centroid is estimated as:
     *   passage_centroid + passage_normal * estimated_room_depth
     *
     * where passage_normal points from the known room toward the far side.
     * The depth heuristic (0.15 m) deliberately stays close to the
     * passage rather than guessing a typical room depth -- it is a
     * placeholder handle, not a position estimate, and gets corrected
     * the moment real far-side evidence (a wall, a cluster) arrives.
     *
     * Constraints:
     *   - Spatial deduplication: reuse existing prospective room
     * within 2.0m
     *   - Max 12 prospective rooms total (matches office_clean's 12 rooms)
     *   - Max 2 rooms per passage (near + far side)
     *   - Passage pointer is the persistent primary handle
     *   - Wall ownership alone never promotes the prospective
     *   - Validated far-side cluster evidence may promote it in place
     */
    if (clearStaleFarSideHandle(p_passage_inout, linkState_inout) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: clearStaleFarSideHandle returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }
    if (resolveConfirmedFarSideRoom(p_passage_inout, linkState_inout) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: resolveConfirmedFarSideRoom returned a failure "
                     "status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (createProspectiveFarSideRoom(allRooms_in,
                                     allPassages_in,
                                     p_passage_inout,
                                     linkState_inout) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: createProspectiveFarSideRoom returned a failure "
                     "status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (restoreFarSideHandle(allRooms_in, p_passage_inout, linkState_inout) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: restoreFarSideHandle returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }

    bool passageIsPassable2{};
    if (p_passage_inout->isPassable(passageIsPassable2) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isPassable returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool passageTraversalEvidence2{};
    if (!(passageIsPassable2) &&
        p_passage_inout->getTraversalEvidence(passageTraversalEvidence2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTraversalEvidence returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool passageHasProspectiveRoom3{};
    bool knownSideHasDirection9{};
    if (((passageIsPassable2 || passageTraversalEvidence2)) &&
        !(linkState_inout.confirmedAssociatedRoomCount > 0U) &&
        linkState_inout.knownSide.hasDirection(knownSideHasDirection9) !=
            semantic::KnownSideProvenanceStatus::
                KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (((passageIsPassable2 || passageTraversalEvidence2) &&
         (linkState_inout.confirmedAssociatedRoomCount > 0U ||
          knownSideHasDirection9)) &&
        p_passage_inout->hasProspectiveRoom(passageHasProspectiveRoom3) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasProspectiveRoom returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool knownSideHasDirection10{};
    if (((passageIsPassable2 || passageTraversalEvidence2)) &&
        !(linkState_inout.confirmedAssociatedRoomCount > 0U) &&
        linkState_inout.knownSide.hasDirection(knownSideHasDirection10) !=
            semantic::KnownSideProvenanceStatus::
                KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if ((passageIsPassable2 || passageTraversalEvidence2) &&
        (linkState_inout.confirmedAssociatedRoomCount > 0U ||
         knownSideHasDirection10) &&
        !passageHasProspectiveRoom3)
    {
        int passageId18{};
        if (p_passage_inout->getId(passageId18) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cerr << "[SemMgr] WARNING: Passage#" << passageId18
                  << " has a confirmed side but no stable far-side handle; "
                     "it is not routable this cycle."
                  << std::endl;
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
