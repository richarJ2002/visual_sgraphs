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
#include <algorithm>
#include <cmath>
#include <limits>

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
                // getFloor cannot fail; continue as before.
            }
            if (p_previousFloor != nullptr && p_previousFloor != this)
            {
                if (p_previousFloor->detachRoom(p_room) !=
                    FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    // detachRoom cannot fail; continue as before.
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
            // getFloor cannot fail; continue as before.
        }
        if (p_oldRoom != nullptr &&
            std::find(newRooms.begin(), newRooms.end(), p_oldRoom) ==
                newRooms.end() &&
            p_oldRoomFloor == this)
        {
            if (p_oldRoom->setFloor(nullptr) != RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setFloor cannot fail; continue as before.
            }
        }
    }

    for (Room *p_newRoom : newRooms)
    {
        if (p_newRoom->setFloor(this) != RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setFloor cannot fail; continue as before.
        }
    }

    return FloorStatus::FLOOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
