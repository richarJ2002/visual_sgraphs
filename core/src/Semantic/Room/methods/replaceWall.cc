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

bool Room::replaceWall(geometric::Plane *p_retiredWall_in,
                       geometric::Plane *p_retainedWall_in)
{
    if (p_retiredWall_in == nullptr || p_retainedWall_in == nullptr ||
        p_retiredWall_in == p_retainedWall_in)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(wallsMutex);

    bool                            replacedRetiredWall = false;
    std::vector<geometric::Plane *> rebuiltWalls;
    rebuiltWalls.reserve(walls.size());

    for (geometric::Plane *p_existingWall : walls)
    {
        geometric::Plane *p_candidateWall = p_existingWall;

        if (p_existingWall == p_retiredWall_in)
        {
            p_candidateWall     = p_retainedWall_in;
            replacedRetiredWall = true;
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

    if (replacedRetiredWall)
    {
        walls.swap(rebuiltWalls);
    }

    return replacedRetiredWall;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
