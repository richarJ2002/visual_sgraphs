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
 * @file            withDirection.cc
 *
 * @brief           Implements Passage::addTraversalObservation()
 *                  (withDirection), declared in Semantic/Passage.h.
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

PassageStatus Passage::addTraversalObservation(TraversalDirection direction_in)
{
    std::lock_guard<std::mutex> lock(typeMutex);

    std::size_t *p_counter = &traversalUnknownCount;
    if (direction_in == TraversalDirection::KNOWN_TO_FAR)
    {
        p_counter = &traversalKnownToFarCount;
    }
    else if (direction_in == TraversalDirection::FAR_TO_KNOWN)
    {
        p_counter = &traversalFarToKnownCount;
    }

    if (*p_counter < std::numeric_limits<std::size_t>::max())
    {
        ++(*p_counter);
    }

    return PassageStatus::PASSAGE_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
