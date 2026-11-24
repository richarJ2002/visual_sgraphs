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
 * @file            measureFiniteCloudOverlap.cc
 *
 * @brief           Implements measureFiniteCloudOverlap(), declared in
 *                  geometry_helpers/private_functions/private_functions.h.
 */

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <variant>

#include <pcl/common/point_tests.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus measureFiniteCloudOverlap(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_observedCloud_in,
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_mappedCloud_in,
    std::size_t                                         maximumSampleCount_in,
    double       maximumFiniteCloudDistance_in,
    std::size_t &sampledPointCount_out,
    std::size_t &overlappingPointCount_out,
    double      &minimumCloudDistance_out)
{
    pcl::KdTreeFLANN<pcl::PointXYZRGBA> mappedCloudSearch;

    mappedCloudSearch.setInputCloud(p_mappedCloud_in);

    const std::size_t samplingStride = std::max<std::size_t>(
        1,
        p_observedCloud_in->size() / maximumSampleCount_in);

    sampledPointCount_out     = 0;
    overlappingPointCount_out = 0;

    minimumCloudDistance_out = std::numeric_limits<double>::max();

    std::vector<int> nearestPointIndex(1);

    std::vector<float> nearestSquaredDistance(1);

    for (std::size_t pointIndex = 0; pointIndex < p_observedCloud_in->size();
         pointIndex += samplingStride)
    {
        const pcl::PointXYZRGBA &queryPoint =
            p_observedCloud_in->points[pointIndex];

        if (!pcl::isFinite(queryPoint))
        {
            continue;
        }

        sampledPointCount_out++;

        const int neighbourCount =
            mappedCloudSearch.nearestKSearch(queryPoint,
                                             1,
                                             nearestPointIndex,
                                             nearestSquaredDistance);

        if (neighbourCount <= 0)
        {
            continue;
        }

        const double cloudDistance =
            std::sqrt(static_cast<double>(nearestSquaredDistance.front()));

        minimumCloudDistance_out =
            std::min(minimumCloudDistance_out, cloudDistance);

        if (cloudDistance <= maximumFiniteCloudDistance_in)
        {
            overlappingPointCount_out++;
        }
    }

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
