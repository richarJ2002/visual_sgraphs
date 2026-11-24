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
 * @file            isWallOwnedByOtherConfirmedRoom.cc
 *
 * @brief           Implements
 * SemanticsManager::isWallOwnedByOtherConfirmedRoom(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::isWallOwnedByOtherConfirmedRoom(
    semantic::Room   *p_room_in,
    geometric::Plane *p_wall_in,
    bool             &isOwned_out)
{
    isOwned_out = false;
    std::vector<semantic::Room *> atlasAllRooms{};
    if (getAllRoomsInIdOrder(atlasAllRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRoomsInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    for (vs_graphs::core::semantic::Room *p_other : atlasAllRooms)
    {
        bool otherIsBad{};
        if (!(p_other == nullptr) &&
            p_other->isBad(otherIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room::RoomVariant otherRoomVariant{};
        if (!(p_other == nullptr || otherIsBad || p_other == p_room_in) &&
            p_other->getRoomVariant(otherRoomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_other == nullptr || otherIsBad || p_other == p_room_in ||
            otherRoomVariant ==
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            continue;
        }
        std::vector<geometric::Plane *> otherWalls{};
        if (p_other->getWalls(otherWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (std::find(otherWalls.begin(), otherWalls.end(), p_wall_in) !=
            otherWalls.end())
        {
            isOwned_out = true;
            break;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
