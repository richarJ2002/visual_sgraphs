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
 * @file            computeSurfaceOffset_m.cc
 *
 * @brief           Implements computeSurfaceOffset_m(), declared in
 *                  passage_detection/slab_profiles/private_functions/
 *                  private_functions.h.
 */

#include "private_functions.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace vs_graphs
{
namespace core
{

double computeSurfaceOffset_m(const WallSlabProfile &wallProfile_in,
                              std::size_t            freeIndex_in,
                              std::size_t            neighbourIndex_in,
                              bool                   hasNeighbour_in)
{
    const double cellSize_m = wallProfile_in.query.cellSize_m;
    if (!hasNeighbour_in ||
        wallProfile_in.cellStates[neighbourIndex_in] != SlabCellState::OCCUPIED)
    {
        return 0.5 * cellSize_m;
    }
    const double freeDistance_m =
        static_cast<double>(wallProfile_in.cellDistances_m[freeIndex_in]);
    const double wallDistance_m =
        static_cast<double>(wallProfile_in.cellDistances_m[neighbourIndex_in]);
    const double distanceStep_m = freeDistance_m - wallDistance_m;
    if (!std::isfinite(distanceStep_m) || distanceStep_m <= 0.0)
    {
        return 0.5 * cellSize_m;
    }
    return cellSize_m * std::clamp(freeDistance_m / distanceStep_m, 0.0, 1.0);
}

} /* namespace core */
} /* namespace vs_graphs */
