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
 * @file            computeObservedApertureFraction.cc
 *
 * @brief           Implements computeObservedApertureFraction(), declared in
 *                  passage_detection/slab_profiles/private_functions/
 *                  private_functions.h.
 */

#include "private_functions.h"

#include <cstddef>

namespace vs_graphs
{
namespace core
{

double computeObservedApertureFraction(const WallSlabProfile &wallProfile_in,
                                       double       doorBandMinimum_m_in,
                                       double       doorBandMaximum_m_in,
                                       double       alongStart_m_in,
                                       double       alongEnd_m_in,
                                       std::size_t &freeCellCount_out,
                                       std::size_t &bandCellCount_out)
{
    const WallSlabQuery &wallQuery = wallProfile_in.query;
    std::size_t          cellCount = 0U, observedCount = 0U, freeCount = 0U;
    for (std::size_t row = 0U; row < wallQuery.rowCount; row++)
    {
        const double height_m = (static_cast<double>(row) + 0.5) *
                                wallQuery.cellSize_m * wallQuery.upAxisRise;
        if (height_m < doorBandMinimum_m_in || height_m > doorBandMaximum_m_in)
        {
            continue;
        }
        for (std::size_t column = 0U; column < wallQuery.columnCount; column++)
        {
            const double along_m =
                (static_cast<double>(column) + 0.5) * wallQuery.cellSize_m;
            if (along_m < alongStart_m_in || along_m > alongEnd_m_in)
            {
                continue;
            }
            cellCount++;
            if (wallProfile_in.cellStates[row * wallQuery.columnCount +
                                          column] != SlabCellState::UNKNOWN)
            {
                observedCount++;
            }
            if (wallProfile_in.cellStates[row * wallQuery.columnCount +
                                          column] == SlabCellState::FREE)
            {
                freeCount++;
            }
        }
    }
    freeCellCount_out = freeCount;
    bandCellCount_out = cellCount;
    return cellCount == 0U ? 0.0
                           : static_cast<double>(observedCount) /
                                 static_cast<double>(cellCount);
}

} /* namespace core */
} /* namespace vs_graphs */
