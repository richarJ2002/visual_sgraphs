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
 * @file            projectPlaneBounds.cc
 *
 * @brief           Implements projectPlaneBounds(), declared in
 *                  Utils/Utils/private_functions.h.
 */

#include "Utils/Utils/private_functions.h"

#include <algorithm>
#include <cmath>

#include <pcl/common/point_tests.h>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus projectPlaneBounds(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_planeCloud_in,
    const Eigen::Vector3d                              &tangentU_World_in,
    const Eigen::Vector3d                              &tangentV_World_in,
    ProjectedPlaneBounds                               &planeBounds_out)
{
    ProjectedPlaneBounds bounds;

    if (p_planeCloud_in == nullptr)
    {
        planeBounds_out = bounds;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    for (const pcl::PointXYZRGBA &point_World_m : p_planeCloud_in->points)
    {
        if (!pcl::isFinite(point_World_m))
        {
            continue;
        }

        const Eigen::Vector3d position_World_m(point_World_m.x,
                                               point_World_m.y,
                                               point_World_m.z);

        const double coordinateU_m = tangentU_World_in.dot(position_World_m);
        const double coordinateV_m = tangentV_World_in.dot(position_World_m);

        bounds.minimumU_m = std::min(bounds.minimumU_m, coordinateU_m);
        bounds.maximumU_m = std::max(bounds.maximumU_m, coordinateU_m);
        bounds.minimumV_m = std::min(bounds.minimumV_m, coordinateV_m);
        bounds.maximumV_m = std::max(bounds.maximumV_m, coordinateV_m);
        bounds.isValid    = true;
    }

    planeBounds_out = bounds;
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
