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

bool Room::replacePassageAssociation(
    vs_graphs::core::semantic::Passage *p_retiredPassage_in,
    vs_graphs::core::semantic::Passage *p_retainedPassage_in)
{
    if (p_retiredPassage_in == nullptr || p_retainedPassage_in == nullptr ||
        p_retiredPassage_in == p_retainedPassage_in)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mapMutex);

    bool replacedAssociation = false;
    std::vector<vs_graphs::core::semantic::Passage *> rebuiltPassages;
    rebuiltPassages.reserve(doorways.size());

    for (vs_graphs::core::semantic::Passage *p_existingPassage : doorways)
    {
        vs_graphs::core::semantic::Passage *p_candidatePassage =
            p_existingPassage;

        if (p_existingPassage == p_retiredPassage_in)
        {
            p_candidatePassage  = p_retainedPassage_in;
            replacedAssociation = true;
        }

        if (p_candidatePassage == nullptr ||
            std::find(rebuiltPassages.begin(),
                      rebuiltPassages.end(),
                      p_candidatePassage) != rebuiltPassages.end())
        {
            continue;
        }

        rebuiltPassages.push_back(p_candidatePassage);
    }

    if (replacedAssociation)
    {
        doorways.swap(rebuiltPassages);
    }

    return replacedAssociation;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
