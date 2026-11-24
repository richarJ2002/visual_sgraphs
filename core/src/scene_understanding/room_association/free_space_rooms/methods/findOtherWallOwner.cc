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
 * @file            findOtherWallOwner.cc
 *
 * @brief           Implements SemanticsManager::findOtherWallOwner(), declared
 * in semantics_manager/objects/SemanticsManagerClass.h.
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

SemanticsManagerStatus
    SemanticsManager::findOtherWallOwner(semantic::Room   *p_room_in,
                                         geometric::Plane *p_wall_in,
                                         semantic::Room  *&p_wallOwner_out)
{
    p_wallOwner_out = nullptr;

    /*
     * A mapped wall surface has one room owner. A room detected on
     * the opposite side must be bounded by its independently observed
     * wall surface rather than sharing this plane object.
     */
    std::vector<vs_graphs::core::semantic::Room *> mappedRooms{};
    if (getAllRoomsInIdOrder(mappedRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRoomsInIdOrder returned a failure "
                     "status; continuing as before.",
                     __func__);
    }

    for (vs_graphs::core::semantic::Room *p_otherRoom : mappedRooms)
    {
        bool otherRoomIsBad{};
        if (!(p_otherRoom == nullptr || p_otherRoom == p_room_in) &&
            p_otherRoom->isBad(otherRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status "
                         "although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_otherRoom == nullptr || p_otherRoom == p_room_in ||
            otherRoomIsBad)
        {
            continue;
        }

        std::vector<vs_graphs::core::geometric::Plane *> otherRoomWalls{};
        if (p_otherRoom->getWalls(otherRoomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status "
                         "although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (std::find(otherRoomWalls.begin(),
                      otherRoomWalls.end(),
                      p_wall_in) != otherRoomWalls.end())
        {
            p_wallOwner_out = p_otherRoom;
            break;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
