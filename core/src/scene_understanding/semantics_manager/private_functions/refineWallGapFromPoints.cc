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
 * @file            refineWallGapFromPoints.cc
 *
 * @brief           Implements refineWallGapFromPoints(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "private_functions.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus refineWallGapFromPoints(
    const WallSlabQuery                                 &wallQuery_in,
    const pcl::PointCloud<pcl::PointXYZRGBA>            &wallCloud_in,
    const types::SystemParams::SemSeg::PassageDetection &passageParameters_in,
    WallGap                                             &wallGap_inout)
{
    const double cellSize_m = wallQuery_in.cellSize_m;
    const double doorBandMinimum_m =
        static_cast<double>(passageParameters_in.doorBandMinHeight_m);
    const double doorBandMaximum_m =
        static_cast<double>(passageParameters_in.doorBandMaxHeight_m);
    if (!(cellSize_m > 0.0) || !(wallQuery_in.upAxisRise > 1e-3))
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /*
     * The slab gap is only a coarse aperture (0.2 m TSDF voxels and pose
     * drift shift and widen it), so the edges are searched a little beyond.
     */
    constexpr double searchMargin_m = 0.3;
    const double searchStart_m = wallGap_inout.alongStart_m - searchMargin_m;
    const double searchEnd_m   = wallGap_inout.alongEnd_m + searchMargin_m;
    const double coarseCentre_m =
        0.5 * (wallGap_inout.alongStart_m + wallGap_inout.alongEnd_m);

    /* Along positions of the wall points of each door-band row. */
    const std::size_t rowCount = static_cast<std::size_t>(
        std::ceil((doorBandMaximum_m - doorBandMinimum_m) / cellSize_m));
    std::vector<std::vector<double>>       rowAlongs_m(rowCount);
    std::vector<std::pair<double, double>> aboveBandPoints{};
    for (const pcl::PointXYZRGBA &wallPoint : wallCloud_in.points)
    {
        if (!pcl::isFinite(wallPoint))
        {
            continue;
        }
        const Eigen::Vector3d offset_world_m =
            Eigen::Vector3d(wallPoint.x, wallPoint.y, wallPoint.z) -
            wallQuery_in.origin_world_m;
        const double along_m = offset_world_m.dot(wallQuery_in.alongAxis_world);
        const double height_m = offset_world_m.dot(wallQuery_in.upAxis_world) *
                                wallQuery_in.upAxisRise;
        if (along_m < searchStart_m || along_m > searchEnd_m)
        {
            continue;
        }
        if (height_m > doorBandMaximum_m)
        {
            aboveBandPoints.emplace_back(along_m, height_m);
        }
        else if (height_m >= doorBandMinimum_m)
        {
            const std::size_t row =
                std::min(rowCount - 1U,
                         static_cast<std::size_t>(
                             (height_m - doorBandMinimum_m) / cellSize_m));
            rowAlongs_m[row].push_back(along_m);
        }
    }

    /*
     * Per row, the wall ends either side of the coarse gap centre: the
     * nearest point backed by a dense run of wall points, so isolated
     * points (depth-edge noise inside the opening) do not narrow it. A row
     * without a wall end inside the search window is skipped.
     */
    constexpr double      wallRunLength_m      = 0.05;
    constexpr std::size_t minimumWallRunPoints = 3U;
    std::vector<double>   rowStarts_m{}, rowEnds_m{};
    for (std::vector<double> &alongs_m : rowAlongs_m)
    {
        std::sort(alongs_m.begin(), alongs_m.end());
        const std::ptrdiff_t firstRight =
            std::upper_bound(alongs_m.begin(), alongs_m.end(), coarseCentre_m) -
            alongs_m.begin();
        const std::ptrdiff_t pointCount =
            static_cast<std::ptrdiff_t>(alongs_m.size());
        std::ptrdiff_t left = firstRight - 1;
        while (left >= 0 &&
               (left + 1 < static_cast<std::ptrdiff_t>(minimumWallRunPoints) ||
                alongs_m[left] - alongs_m[left + 1 -
                                          static_cast<std::ptrdiff_t>(
                                              minimumWallRunPoints)] >
                    wallRunLength_m))
        {
            left--;
        }
        std::ptrdiff_t right = firstRight;
        while (
            right < pointCount &&
            (right + static_cast<std::ptrdiff_t>(minimumWallRunPoints) >
                 pointCount ||
             alongs_m[right +
                      static_cast<std::ptrdiff_t>(minimumWallRunPoints) - 1] -
                     alongs_m[right] >
                 wallRunLength_m))
        {
            right++;
        }
        if (left < 0 || right >= pointCount)
        {
            continue;
        }
        rowStarts_m.push_back(alongs_m[left]);
        rowEnds_m.push_back(alongs_m[right]);
    }
    if (rowStarts_m.empty() ||
        static_cast<double>(rowStarts_m.size()) <
            static_cast<double>(
                passageParameters_in.minimumBoundedBandFraction) *
                static_cast<double>(rowCount))
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const auto median = [](std::vector<double> values)
    {
        const std::size_t middle = values.size() / 2U;
        std::nth_element(values.begin(), values.begin() + middle, values.end());
        return values[middle];
    };
    const double alongStart_m = median(rowStarts_m);
    const double alongEnd_m   = median(rowEnds_m);
    const double width_m      = alongEnd_m - alongStart_m;
    if (!(width_m > 0.0))
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /*
     * Lintel: per cell-wide column of the opening the lowest wall point above
     * the door band, then the median over the columns that have one.
     */
    const std::size_t columnCount = static_cast<std::size_t>(
        std::max(1.0, std::floor(width_m / cellSize_m)));
    std::vector<double> columnTops_m(columnCount,
                                     std::numeric_limits<double>::infinity());
    for (const std::pair<double, double> &aboveBandPoint : aboveBandPoints)
    {
        if (aboveBandPoint.first > alongStart_m &&
            aboveBandPoint.first < alongEnd_m)
        {
            const std::size_t column =
                std::min(columnCount - 1U,
                         static_cast<std::size_t>(
                             (aboveBandPoint.first - alongStart_m) / width_m *
                             static_cast<double>(columnCount)));
            columnTops_m[column] =
                std::min(columnTops_m[column], aboveBandPoint.second);
        }
    }
    std::vector<double> observedTops_m{};
    for (const double columnTop_m : columnTops_m)
    {
        if (std::isfinite(columnTop_m))
        {
            observedTops_m.push_back(columnTop_m);
        }
    }
    if (2U * observedTops_m.size() >= columnCount)
    {
        wallGap_inout.topHeight_m = median(observedTops_m);
    }

    wallGap_inout.alongStart_m = alongStart_m;
    wallGap_inout.alongEnd_m   = alongEnd_m;
    wallGap_inout.width_m      = width_m;
    wallGap_inout.centre_world_m =
        wallQuery_in.origin_world_m +
        0.5 * (alongStart_m + alongEnd_m) * wallQuery_in.alongAxis_world +
        (0.5 * wallGap_inout.topHeight_m / wallQuery_in.upAxisRise) *
            wallQuery_in.upAxis_world;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
