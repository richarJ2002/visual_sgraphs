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
 * @file            hasWall.cc
 *
 * @brief           Implements Room::hasWall(), declared in
 *                  scene_graph/room/objects/RoomClass.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "scene_understanding/scene_graph/room/objects/RoomClass.h"
#include <algorithm>
#include <mutex>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::hasWall(vs_graphs::core::geometric::Plane *p_wall_in,
                         bool &isWallInRoom_out) const
{
    if (p_wall_in == nullptr)
    {
        return RoomStatus::ROOM_STATUS_INVALID_ARGUMENT;
    }

    /* wallPlanes is written under wallsMutex, so search it under the same. */
    std::lock_guard<std::mutex> wallsLock(wallsMutex);

    isWallInRoom_out =
        std::find(wallPlanes.begin(), wallPlanes.end(), p_wall_in) !=
        wallPlanes.end();

    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
