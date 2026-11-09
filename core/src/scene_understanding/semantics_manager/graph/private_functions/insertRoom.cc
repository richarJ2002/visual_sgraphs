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
 * @file            insertRoom.cc
 *
 * @brief           Implements insertRoom(), declared in
 *                  semantics_manager/graph/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{

void insertRoom(std::map<int, std::unique_ptr<semantic::Room>> &roomsById_inout,
                std::unique_ptr<semantic::Room>                 p_room_in,
                unsigned long                                   mapId_in,
                RetiredNodes &retiredNodes_inout)
{
    if (p_room_in == nullptr)
    {
        return;
    }

    int roomId{};
    if (p_room_in->getId(roomId) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Rooms are keyed by id: a room whose id another room already holds is
     * not added, and the first room keeps the id. try_emplace leaves the
     * pointer untouched when the key exists, so the room can be retired. */
    const std::pair<std::map<int, std::unique_ptr<semantic::Room>>::iterator,
                    bool>
        roomInsertion =
            roomsById_inout.try_emplace(roomId, std::move(p_room_in));
    if (!roomInsertion.second)
    {
        RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                    "%s: room id %d is already taken in map %lu; the room "
                    "was not added.",
                    __func__,
                    roomId,
                    mapId_in);
        retiredNodes_inout.rooms.push_back(std::move(p_room_in));
    }
}

} /* namespace core */
} /* namespace vs_graphs */
