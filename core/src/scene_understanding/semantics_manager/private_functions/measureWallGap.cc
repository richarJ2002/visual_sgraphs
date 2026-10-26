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

    const auto cellIndex = [columnCount](std::size_t row, std::size_t column)
    { return row * columnCount + column; };
    const auto isFreeCell = [&](std::size_t row, std::size_t column)
    {
        return wallProfile_in.cellStates[cellIndex(row, column)] ==
               SlabCellState::FREE;
    };
    const auto rowCentreHeight_m = [rowHeight_m](std::size_t row)
    { return (static_cast<double>(row) + 0.5) * rowHeight_m; };
    const auto columnCentreAlong_m = [cellSize_m](std::size_t column)
    { return (static_cast<double>(column) + 0.5) * cellSize_m; };

    /*
     * Zero crossing between a free cell and its OCCUPIED neighbour, as an
     * offset from the free cell centre towards the neighbour; half a cell
     * when the neighbour is unknown, missing or the distances are unusable.
     */
    const auto surfaceOffset_m = [&](std::size_t freeIndex,
                                     std::size_t neighbourIndex,
                                     bool        hasNeighbour)
    {
        if (!hasNeighbour || wallProfile_in.cellStates[neighbourIndex] !=
                                 SlabCellState::OCCUPIED)
        {
            return 0.5 * cellSize_m;
        }
        const double freeDistance_m =
            static_cast<double>(wallProfile_in.cellDistances_m[freeIndex]);
        const double wallDistance_m =
            static_cast<double>(wallProfile_in.cellDistances_m[neighbourIndex]);
        const double distanceStep_m = freeDistance_m - wallDistance_m;
        if (!std::isfinite(distanceStep_m) || distanceStep_m <= 0.0)
        {
            return 0.5 * cellSize_m;
        }
        return cellSize_m *
               std::clamp(freeDistance_m / distanceStep_m, 0.0, 1.0);
    };

    const double doorBandMinimum_m =
        static_cast<double>(passageParameters_in.doorBandMinHeight_m);
    const double doorBandMaximum_m =
        static_cast<double>(passageParameters_in.doorBandMaxHeight_m);
    std::vector<std::size_t> doorBandRows{};
    for (std::size_t row = 0U; row < rowCount; row++)
    {
        const double rowHeight = rowCentreHeight_m(row);
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
            if (!isFreeCell(seedRow, seedColumn) ||
                componentLabels[cellIndex(seedRow, seedColumn)] != 0U)
            {
                continue;
            }

            /* 4-connected flood fill over FREE cells of the whole slab. */
            const std::size_t        componentLabel = ++componentCount;
            std::vector<std::size_t> openCells{cellIndex(seedRow, seedColumn)};
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
                const auto visit =
                    [&](std::size_t nextRow, std::size_t nextColumn)
                {
                    const std::size_t nextIndex =
                        cellIndex(nextRow, nextColumn);
                    if (isFreeCell(nextRow, nextColumn) &&
                        componentLabels[nextIndex] == 0U)
                    {
                        componentLabels[nextIndex] = componentLabel;
                        openCells.push_back(nextIndex);
                    }
                };
                if (row > 0U)
                {
                    visit(row - 1U, column);
                }
                if (row + 1U < rowCount)
                {
                    visit(row + 1U, column);
                }
                if (column > 0U)
                {
                    visit(row, column - 1U);
                }
                if (column + 1U < columnCount)
                {
                    visit(row, column + 1U);
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
                    if (componentLabels[cellIndex(row, column)] ==
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
                    columnCentreAlong_m(firstColumn) -
                    surfaceOffset_m(cellIndex(row, firstColumn),
                                    firstColumn > 0U
                                        ? cellIndex(row, firstColumn - 1U)
                                        : 0U,
                                    firstColumn > 0U);
                const double end_m =
                    columnCentreAlong_m(lastColumn) +
                    surfaceOffset_m(cellIndex(row, lastColumn),
                                    lastColumn + 1U < columnCount
                                        ? cellIndex(row, lastColumn + 1U)
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

            const auto median = [](std::vector<double> values)
            {
                const std::size_t middle = values.size() / 2U;
                std::nth_element(values.begin(),
                                 values.begin() + middle,
                                 values.end());
                return values[middle];
            };
            std::nth_element(runCentreColumns.begin(),
                             runCentreColumns.begin() +
                                 runCentreColumns.size() / 2U,
                             runCentreColumns.end());
            const std::size_t lintelColumn =
                runCentreColumns[runCentreColumns.size() / 2U];

            /* Lintel: highest component cell of the median run column. */
            std::size_t lintelRow = topRow;
            while (lintelRow > bottomRow &&
                   componentLabels[cellIndex(lintelRow, lintelColumn)] !=
                       componentLabel)
            {
                lintelRow--;
            }
            const double topHeight_m =
                rowCentreHeight_m(lintelRow) +
                upAxisRise * surfaceOffset_m(
                                 cellIndex(lintelRow, lintelColumn),
                                 lintelRow + 1U < rowCount
                                     ? cellIndex(lintelRow + 1U, lintelColumn)
                                     : 0U,
                                 lintelRow + 1U < rowCount);

            WallGap wallGap{};
            wallGap.width_m      = median(rowWidths_m);
            wallGap.alongStart_m = median(rowStarts_m);
            wallGap.alongEnd_m   = median(rowEnds_m);
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
