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
 * @file            countApertureWallPoints.cc
 *
 * @brief           Implements countApertureWallPoints(), declared in
 *                  passage_detection/slab_profiles/private_functions/
 *                  private_functions.h.
 */

#include "private_functions.h"

#include <cstddef>

#include <Eigen/Core>
#include <pcl/common/point_tests.h>

namespace vs_graphs
{
namespace core
{

std::size_t countApertureWallPoints(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_wallCloud_in,
    const WallSlabProfile                              &wallProfile_in,
    double                                              doorBandMinimum_m_in,
    double                                              doorBandMaximum_m_in,
    double                                              alongStart_m_in,
    double                                              alongEnd_m_in)
{
    const WallSlabQuery &wallQuery  = wallProfile_in.query;
    std::size_t          pointCount = 0U;
    if (p_wallCloud_in == nullptr)
    {
        return pointCount;
    }
    for (const pcl::PointXYZRGBA &supportPoint : p_wallCloud_in->points)
    {
        const Eigen::Vector3d offset_world_m =
            Eigen::Vector3d(supportPoint.x, supportPoint.y, supportPoint.z) -
            wallQuery.origin_world_m;
        const double along_m = offset_world_m.dot(wallQuery.alongAxis_world);
        const double height_m =
            offset_world_m.dot(wallQuery.upAxis_world) * wallQuery.upAxisRise;
        if (!pcl::isFinite(supportPoint) || along_m < 0.0 ||
            along_m < alongStart_m_in || along_m > alongEnd_m_in ||
            height_m < doorBandMinimum_m_in || height_m > doorBandMaximum_m_in)
        {
            continue;
        }
        const std::size_t column =
            static_cast<std::size_t>(along_m / wallQuery.cellSize_m);
        const std::size_t row = static_cast<std::size_t>(
            height_m / (wallQuery.cellSize_m * wallQuery.upAxisRise));
        if (column < wallQuery.columnCount && row < wallQuery.rowCount &&
            wallProfile_in.cellStates[row * wallQuery.columnCount + column] !=
                SlabCellState::FREE)
        {
            pointCount++;
        }
    }
    return pointCount;
}

} /* namespace core */
} /* namespace vs_graphs */
