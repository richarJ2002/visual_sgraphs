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
 * @file            refreshClusterRoomGeometry.cc
 *
 * @brief           Implements SemanticsManager::refreshClusterRoomGeometry(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/objects/SemanticsManagerClass.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::refreshClusterRoomGeometry(
    semantic::Room                  *p_room_in,
    std::vector<geometric::Plane *> &roomWalls_out)
{
    /* Find all the walls in a room */
    std::vector<geometric::Plane *> wallsBeforeConsolidation{};
    if (p_room_in->getWalls(wallsBeforeConsolidation) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    roomWalls_out = wallsBeforeConsolidation;

    /*!
     * Consolidate provisional single-wall structural elements whose wall
     * has now been absorbed into the cluster-backed room.
     *
     * @note            This is deliberately more restrictive than the old
     *                  centroid-only reAssociateRooms() implementation.
     */
    if (utils::utils::Utils::consolidateProvisionalRooms(p_room_in, p_atlas) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: consolidateProvisionalRooms returned a failure status "
            "although it cannot fail; continuing as before.",
            __func__);
    }

    /* Remove invalid relationships from the room's persistent graph. */
    std::size_t roomRemovedWallCount{};
    if (p_room_in->removeInvalidWalls(roomRemovedWallCount) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: removeInvalidWalls returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<geometric::Plane *> wallsAfterInvalidRemoval{};
    if (p_room_in->getWalls(wallsAfterInvalidRemoval) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    roomWalls_out = wallsAfterInvalidRemoval;

    /*!
     * The semantic room centre comes from computeRoomCentreFromWalls():
     * the centre of the floor box spanned by the walls' extents, or the
     * inward-pushed wall-centroid mean for walls that span no box. Rooms
     * without walls keep their creation centroid until walls arrive:
     * free-space evidence places a room once, at creation, and never
     * maintains it.
     */
    std::optional<Eigen::Vector3d> wallCentre_world_m{};
    if (computeRoomCentreFromWalls(p_room_in,
                                   roomWalls_out,
                                   wallCentre_world_m) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeRoomCentreFromWalls returned a failure "
                     "status although it cannot fail; continuing as "
                     "before.",
                     __func__);
    }
    if (wallCentre_world_m.has_value())
    {
        /*!
         * Damped update, not a snap. This centroid feeds two decisions
         * that REMOVE walls -- isWallFaceForeignToRoom's foreign-face
         * check (enforcePassageSideInvariant) and the passage far-side
         * router -- which change roomWalls, which changes the centre
         * computed above. An undamped snap closes an unstable feedback
         * loop with no damping: remove a wall -> centroid shifts ->
         * another wall's side test flips -> remove that too -> centroid
         * shifts further. Live-observed 2026-09-04 crossing Office 6's
         * passage: the same handful of walls near the doorway repeatedly
         * removed and re-admitted, tens of times in a row, a previously
         * COMPLETE room's boundary never settling. Blending in a minority
         * weight of the fresh centre still lets the centroid track
         * genuinely new evidence over several cycles, but one cycle's
         * wall churn can no longer swing it far enough to flip another
         * wall's side test.
         */
        Eigen::Vector3d previousCentroid_world_m{};
        if (p_room_in->getCentroid(previousCentroid_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        constexpr double      centroidDampingWeight = 0.25;
        const Eigen::Vector3d dampedCentroid_world_m =
            previousCentroid_world_m.allFinite()
                ? (centroidDampingWeight * (*wallCentre_world_m) +
                   (1.0 - centroidDampingWeight) * previousCentroid_world_m)
                : *wallCentre_world_m;

        if (p_room_in->setCentroid(dampedCentroid_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCentroid returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
