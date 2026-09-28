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

bool Floor::replaceRoom(Room *p_retiredRoom_inout, Room *p_retainedRoom_inout)
{
    if (p_retiredRoom_inout == nullptr || p_retainedRoom_inout == nullptr ||
        p_retiredRoom_inout == p_retainedRoom_inout)
    {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(roomsMutex);
        if (std::find(rooms.begin(), rooms.end(), p_retiredRoom_inout) ==
            rooms.end())
        {
            return false;
        }
    }

    Floor *p_previousRetainedFloor = p_retainedRoom_inout->getFloor();
    if (p_previousRetainedFloor != nullptr && p_previousRetainedFloor != this)
    {
        p_previousRetainedFloor->detachRoom(p_retainedRoom_inout);
    }

    bool                replacedRetiredRoom = false;
    std::vector<Room *> rebuiltRooms;
    {
        std::lock_guard<std::mutex> lock(roomsMutex);
        rebuiltRooms.reserve(rooms.size());

        for (Room *p_existingRoom : rooms)
        {
            Room *p_candidateRoom = p_existingRoom;

            if (p_existingRoom == p_retiredRoom_inout)
            {
                p_candidateRoom     = p_retainedRoom_inout;
                replacedRetiredRoom = true;
            }

            if (p_candidateRoom == nullptr ||
                std::find(rebuiltRooms.begin(),
                          rebuiltRooms.end(),
                          p_candidateRoom) != rebuiltRooms.end())
            {
                continue;
            }

            rebuiltRooms.push_back(p_candidateRoom);
        }

        if (replacedRetiredRoom)
        {
            rooms.swap(rebuiltRooms);
        }
    }

    if (replacedRetiredRoom)
    {
        if (p_retiredRoom_inout->getFloor() == this)
        {
            p_retiredRoom_inout->setFloor(nullptr);
        }
        p_retainedRoom_inout->setFloor(this);
    }

    return replacedRetiredRoom;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
