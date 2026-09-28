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

void Room::setDoorways(vs_graphs::core::semantic::Passage *p_passage_in)
{
    if (p_passage_in == nullptr)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mapMutex);

    const bool isAlreadyPresent = std::any_of(
        doorways.begin(),
        doorways.end(),
        [p_passage_in](vs_graphs::core::semantic::Passage *p_existingPassage)
        {
            return p_existingPassage != nullptr &&
                   p_existingPassage->getId() == p_passage_in->getId();
        });

    if (!isAlreadyPresent)
    {
        doorways.push_back(p_passage_in);
    }
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
