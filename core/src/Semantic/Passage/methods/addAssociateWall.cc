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
 * @file            addAssociateWall.cc
 *
 * @brief           Implements Passage::addAssociateWall(), declared in
 *                  Semantic/Passage.h.
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

PassageStatus
    Passage::addAssociateWall(vs_graphs::core::geometric::Plane *p_wall_in)
{
    if (p_wall_in == nullptr)
    {
        return PassageStatus::PASSAGE_STATUS_SUCCESS;
    }

    std::lock_guard<std::mutex> lock(geometryMutex);

    const bool alreadyPresent = std::any_of(
        associateWalls.begin(),
        associateWalls.end(),
        [p_wall_in](vs_graphs::core::geometric::Plane *p_existingWall)
        { return p_existingWall == p_wall_in; });

    if (!alreadyPresent)
    {
        associateWalls.push_back(p_wall_in);
    }

    return PassageStatus::PASSAGE_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
