/**
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
 * @file            countMapSnapshotsWithId.cc
 *
 * @brief           Implements countMapSnapshotsWithId(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <cstddef>

namespace ORB_SLAM3
{
namespace semantic
{

std::size_t countMapSnapshotsWithId(const SemanticGraphSnapshot &snapshot_in,
                                    long unsigned int            mapId_in)
{
    std::size_t matchCount = 0U;
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        if (mapSnapshot.mapId == mapId_in)
        {
            ++matchCount;
        }
    }
    return matchCount;
}

} // namespace semantic
} // namespace ORB_SLAM3
