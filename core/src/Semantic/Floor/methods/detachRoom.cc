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

FloorStatus Floor::detachRoom(Room *p_room_inout)
{
    if (p_room_inout == nullptr)
    {
        return FloorStatus::FLOOR_STATUS_SUCCESS;
    }

    {
        std::lock_guard<std::mutex> lock(roomsMutex);
        rooms.erase(std::remove(rooms.begin(), rooms.end(), p_room_inout),
                    rooms.end());
    }

    Floor *p_room_inoutFloor = nullptr;
    if (p_room_inout->getFloor(p_room_inoutFloor) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getFloor cannot fail; continue as before.
    }
    if (p_room_inoutFloor == this)
    {
        if (p_room_inout->setFloor(nullptr) != RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setFloor cannot fail; continue as before.
        }
    }

    return FloorStatus::FLOOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
