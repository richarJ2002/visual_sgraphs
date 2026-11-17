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
 * @file            measureWallGap.cc
 *
 * @brief           Implements measureWallGap(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "private_functions.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus measureWallGap(
    const WallSlabProfile                               &wallProfile_in,
    const types::SystemParams::SemSeg::PassageDetection &passageParameters_in,
    std::vector<WallGap>                                &wallGaps_out)
{
    wallGaps_out.clear();

    const WallSlabQuery &wallQuery   = wallProfile_in.query;
    const std::size_t    columnCount = wallQuery.columnCount;
    const std::size_t    rowCount    = wallQuery.rowCount;
    const double         cellSize_m  = wallQuery.cellSize_m;
    const double         upAxisRise  = wallQuery.upAxisRise;
    if (columnCount == 0U || rowCount == 0U || !(cellSize_m > 0.0) ||
        !(upAxisRise > 1e-3) ||
        wallProfile_in.cellStates.size() != columnCount * rowCount ||
        wallProfile_in.cellDistances_m.size() != columnCount * rowCount)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    const double rowHeight_m = cellSize_m * upAxisRise;

    const double doorBandMinimum_m =
        static_cast<double>(passageParameters_in.doorBandMinHeight_m);
    const double doorBandMaximum_m =
        static_cast<double>(passageParameters_in.doorBandMaxHeight_m);
    std::vector<std::size_t> doorBandRows{};
    for (std::size_t row = 0U; row < rowCount; row++)
    {
        const double rowHeight = (static_cast<double>(row) + 0.5) * rowHeight_m;
        if (rowHeight >= doorBandMinimum_m && rowHeight <= doorBandMaximum_m)
        {
            doorBandRows.push_back(row);
        }
    }
    if (doorBandRows.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Component label per cell; 0 = not yet visited. */
    std::vector<std::size_t> componentLabels(columnCount * rowCount, 0U);
    std::size_t              componentCount = 0U;

    for (const std::size_t seedRow : doorBandRows)
    {
        for (std::size_t seedColumn = 0U; seedColumn < columnCount;
             seedColumn++)
        {
            if (wallProfile_in.cellStates[seedRow * columnCount + seedColumn] !=
                    SlabCellState::FREE ||
                componentLabels[seedRow * columnCount + seedColumn] != 0U)
            {
                continue;
            }

            /* 4-connected flood fill over FREE cells of the whole slab. */
            const std::size_t        componentLabel = ++componentCount;
            std::vector<std::size_t> openCells{seedRow * columnCount +
                                               seedColumn};
            componentLabels[openCells.front()] = componentLabel;
            std::size_t topRow = seedRow, bottomRow = seedRow;
            while (!openCells.empty())
            {
                const std::size_t index = openCells.back();
                openCells.pop_back();
                const std::size_t row    = index / columnCount;
                const std::size_t column = index % columnCount;
                topRow                   = std::max(topRow, row);
                bottomRow                = std::min(bottomRow, row);
                if (row > 0U)
                {
                    visitFreeSlabCell(wallProfile_in,
                                      (row - 1U) * columnCount + column,
                                      componentLabel,
                                      componentLabels,
                                      openCells);
                }
                if (row + 1U < rowCount)
                {
                    visitFreeSlabCell(wallProfile_in,
                                      (row + 1U) * columnCount + column,
                                      componentLabel,
                                      componentLabels,
                                      openCells);
                }
                if (column > 0U)
                {
                    visitFreeSlabCell(wallProfile_in,
                                      row * columnCount + column - 1U,
                                      componentLabel,
                                      componentLabels,
                                      openCells);
                }
                if (column + 1U < columnCount)
                {
                    visitFreeSlabCell(wallProfile_in,
                                      row * columnCount + column + 1U,
                                      componentLabel,
                                      componentLabels,
                                      openCells);
                }
            }

            /*
             * A door-band row is bounded when the component crosses it as one
             * run; its ends are then the wall faces either side.
             */
            std::vector<double>      rowWidths_m{}, rowStarts_m{}, rowEnds_m{};
            std::vector<std::size_t> runCentreColumns{};
            for (const std::size_t row : doorBandRows)
            {
                std::size_t firstColumn = columnCount, lastColumn = 0U;
                std::size_t memberCount = 0U;
                for (std::size_t column = 0U; column < columnCount; column++)
                {
                    if (componentLabels[row * columnCount + column] ==
                        componentLabel)
                    {
                        firstColumn = std::min(firstColumn, column);
                        lastColumn  = std::max(lastColumn, column);
                        memberCount++;
                    }
                }
                if (memberCount == 0U ||
                    memberCount != lastColumn - firstColumn + 1U)
                {
                    continue;
                }
                const double start_m =
                    (static_cast<double>(firstColumn) + 0.5) * cellSize_m -
                    computeSurfaceOffset_m(
                        wallProfile_in,
                        row * columnCount + firstColumn,
                        firstColumn > 0U ? row * columnCount + firstColumn - 1U
                                         : 0U,
                        firstColumn > 0U);
                const double end_m =
                    (static_cast<double>(lastColumn) + 0.5) * cellSize_m +
                    computeSurfaceOffset_m(wallProfile_in,
                                           row * columnCount + lastColumn,
                                           lastColumn + 1U < columnCount
                                               ? row * columnCount +
                                                     lastColumn + 1U
                                               : 0U,
                                           lastColumn + 1U < columnCount);
                rowStarts_m.push_back(start_m);
                rowEnds_m.push_back(end_m);
                rowWidths_m.push_back(end_m - start_m);
                runCentreColumns.push_back((firstColumn + lastColumn) / 2U);
            }

            const double boundedBandFraction =
                static_cast<double>(rowWidths_m.size()) /
                static_cast<double>(doorBandRows.size());
            if (rowWidths_m.empty() ||
                boundedBandFraction <
                    static_cast<double>(
                        passageParameters_in.minimumBoundedBandFraction))
            {
                continue;
            }

            std::nth_element(runCentreColumns.begin(),
                             runCentreColumns.begin() +
                                 runCentreColumns.size() / 2U,
                             runCentreColumns.end());
            const std::size_t lintelColumn =
                runCentreColumns[runCentreColumns.size() / 2U];

            /* Lintel: highest component cell of the median run column. */
            std::size_t lintelRow = topRow;
            while (lintelRow > bottomRow &&
                   componentLabels[lintelRow * columnCount + lintelColumn] !=
                       componentLabel)
            {
                lintelRow--;
            }
            const double topHeight_m =
                (static_cast<double>(lintelRow) + 0.5) * rowHeight_m +
                upAxisRise *
                    computeSurfaceOffset_m(
                        wallProfile_in,
                        lintelRow * columnCount + lintelColumn,
                        lintelRow + 1U < rowCount
                            ? (lintelRow + 1U) * columnCount + lintelColumn
                            : 0U,
                        lintelRow + 1U < rowCount);

            WallGap wallGap{};
            wallGap.width_m      = computeMedianValue(rowWidths_m);
            wallGap.alongStart_m = computeMedianValue(rowStarts_m);
            wallGap.alongEnd_m   = computeMedianValue(rowEnds_m);
            wallGap.bottomHeight_m =
                static_cast<double>(bottomRow) * rowHeight_m;
            wallGap.topHeight_m = topHeight_m;

            if (wallGap.width_m <
                    static_cast<double>(
                        passageParameters_in.minimumOpeningWidth_m) ||
                wallGap.width_m >
                    static_cast<double>(
                        passageParameters_in.maximumOpeningWidth_m) ||
                wallGap.topHeight_m <
                    static_cast<double>(
                        passageParameters_in.minimumOpeningHeight_m) ||
                wallGap.bottomHeight_m >
                    static_cast<double>(
                        passageParameters_in.maximumSillHeight_m))
            {
                continue;
            }

            /* Mid along x mid height, on the wall plane. */
            wallGap.centre_world_m =
                wallQuery.origin_world_m +
                0.5 * (wallGap.alongStart_m + wallGap.alongEnd_m) *
                    wallQuery.alongAxis_world +
                (0.5 * wallGap.topHeight_m / upAxisRise) *
                    wallQuery.upAxis_world;
            wallGaps_out.push_back(wallGap);
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
