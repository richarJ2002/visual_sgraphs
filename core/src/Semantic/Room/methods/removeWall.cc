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

#include "Semantic/Room.h"
#include <algorithm>
#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::removeWall(geometric::Plane *p_wall_in,
                            bool             &wasWallRemoved_out)
{
    if (p_wall_in == nullptr)
    {
        return RoomStatus::ROOM_STATUS_INVALID_ARGUMENT;
    }

    std::lock_guard<std::mutex> lock(wallsMutex);

    const auto wallIt = std::remove(walls.begin(), walls.end(), p_wall_in);
    const bool wasWallRemoved = wallIt != walls.end();
    walls.erase(wallIt, walls.end());

    wasWallRemoved_out = wasWallRemoved;
    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
