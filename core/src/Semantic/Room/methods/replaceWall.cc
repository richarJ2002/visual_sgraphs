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
 * @file            replaceWall.cc
 *
 * @brief           Implements Room::replaceWall(), declared in Semantic/Room.h.
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

RoomStatus Room::replaceWall(geometric::Plane *p_retiredWall_in,
                             geometric::Plane *p_retainedWall_in,
                             bool             &wasWallReplaced_out)
{
    if (p_retiredWall_in == nullptr || p_retainedWall_in == nullptr ||
        p_retiredWall_in == p_retainedWall_in)
    {
        return RoomStatus::ROOM_STATUS_INVALID_ARGUMENT;
    }

    std::lock_guard<std::mutex> lock(wallsMutex);

    bool                            wasRetiredWallReplaced = false;
    std::vector<geometric::Plane *> rebuiltWalls;
    rebuiltWalls.reserve(walls.size());

    for (geometric::Plane *p_existingWall : walls)
    {
        geometric::Plane *p_candidateWall = p_existingWall;

        if (p_existingWall == p_retiredWall_in)
        {
            p_candidateWall        = p_retainedWall_in;
            wasRetiredWallReplaced = true;
        }

        if (p_candidateWall == nullptr ||
            std::find(rebuiltWalls.begin(),
                      rebuiltWalls.end(),
                      p_candidateWall) != rebuiltWalls.end())
        {
            continue;
        }

        rebuiltWalls.push_back(p_candidateWall);
    }

    if (wasRetiredWallReplaced)
    {
        walls.swap(rebuiltWalls);
    }

    wasWallReplaced_out = wasRetiredWallReplaced;
    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
