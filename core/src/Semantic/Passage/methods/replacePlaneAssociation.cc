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

#include "Semantic/Passage.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool Passage::replacePlaneAssociation(geometric::Plane *p_retiredPlane_in,
                                      geometric::Plane *p_retainedPlane_in)
{
    if (p_retiredPlane_in == nullptr || p_retainedPlane_in == nullptr ||
        p_retiredPlane_in == p_retainedPlane_in)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mMutexGeometry);

    bool replacedAssociation = false;

    if (associateDoor == p_retiredPlane_in)
    {
        associateDoor       = p_retainedPlane_in;
        replacedAssociation = true;
    }

    std::vector<geometric::Plane *> rebuiltWalls;
    rebuiltWalls.reserve(associateWalls.size());

    for (geometric::Plane *p_existingWall : associateWalls)
    {
        geometric::Plane *p_candidateWall = p_existingWall;

        if (p_existingWall == p_retiredPlane_in)
        {
            p_candidateWall     = p_retainedPlane_in;
            replacedAssociation = true;
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

    if (replacedAssociation)
    {
        associateWalls.swap(rebuiltWalls);
    }

    return replacedAssociation;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
