/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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

#include "Semantic/Floor.h"
#include "Semantic/Room.h"
#include "Semantic/RoomStatus.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

FloorStatus Floor::setRooms(
    const std::vector<vs_graphs::core::semantic::Room *> &value_in)
{
    std::vector<Room *> newRooms;
    newRooms.reserve(value_in.size());

    for (Room *p_room : value_in)
    {
        if (p_room != nullptr &&
            std::find(newRooms.begin(), newRooms.end(), p_room) ==
                newRooms.end())
        {
            Floor *p_previousFloor = nullptr;
            if (p_room->getFloor(p_previousFloor) !=
                RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getFloor returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_previousFloor != nullptr && p_previousFloor != this)
            {
                if (p_previousFloor->detachRoom(p_room) !=
                    FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: detachRoom returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
            newRooms.push_back(p_room);
        }
    }

    std::vector<Room *> oldRooms;
    {
        std::lock_guard<std::mutex> lock(roomsMutex);
        oldRooms = rooms;
        rooms    = newRooms;
    }

    for (Room *p_oldRoom : oldRooms)
    {
        Floor *p_oldRoomFloor = nullptr;
        if ((p_oldRoom != nullptr &&
             std::find(newRooms.begin(), newRooms.end(), p_oldRoom) ==
                 newRooms.end()) &&
            p_oldRoom->getFloor(p_oldRoomFloor) !=
                RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFloor returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_oldRoom != nullptr &&
            std::find(newRooms.begin(), newRooms.end(), p_oldRoom) ==
                newRooms.end() &&
            p_oldRoomFloor == this)
        {
            if (p_oldRoom->setFloor(nullptr) != RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setFloor returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (Room *p_newRoom : newRooms)
    {
        if (p_newRoom->setFloor(this) != RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setFloor returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    return FloorStatus::FLOOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
