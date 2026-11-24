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
 * @file            findRoomByMapAndId.cc
 *
 * @brief           Implements SemanticsManager::findRoomByMapAndId(), declared
 *                  in SemanticsManager.h.
 */

#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::findRoomByMapAndId(long unsigned int mapId_in,
                                         int               roomId_in,
                                         semantic::Room  *&p_room_out) const
{
    /* A map the graph registry does not hold (no longer in the Atlas) has no
     * room: MAP_NOT_FOUND leaves the room null, as a missing id does. */
    semantic::Room *p_room = nullptr;
    if (getMapRoomById(mapId_in, roomId_in, p_room) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        p_room == nullptr)
    {
        p_room_out = nullptr;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    semantic::Room::RoomVariant roomVariant{};
    if (p_room->getRoomVariant(roomVariant) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomVariant returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    p_room_out =
        roomVariant == semantic::Room::RoomVariant::ROOM ? p_room : nullptr;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
