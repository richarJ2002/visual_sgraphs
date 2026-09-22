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
 * @file            pointcloudDistanceFilter.cc
 *
 * @brief           Implements the
 *                  Utils::pointcloudDistanceFilter(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <algorithm>
#include <iterator>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr Utils::pointcloudDistanceFilter(
    const typename pcl::PointCloud<PointT>::Ptr &p_cloud_in)
{
    // Variables
    double                        distance;
    const std::pair<float, float> thresholds =
        types::SystemParams::getParams()->pointcloud.distanceThresh;
    const float thresholdNear = thresholds.first;
    const float thresholdFar  = thresholds.second;

    // Define the filtered point p_cloud_in object
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(
        new pcl::PointCloud<PointT>());
    filteredCloud->reserve(p_cloud_in->size());

    // Filter the point p_cloud_in
    std::copy_if(p_cloud_in->begin(),
                 p_cloud_in->end(),
                 std::back_inserter(filteredCloud->points),
                 [&](const PointT &p)
                 {
                     /*!
                      * Filter points based on distance along z axis from
                      * sensor.
                      *
                      * @note:      The z axis is pointing away from the
                      *             sensor.
                      */
                     distance = p.z;
                     return distance > thresholdNear && distance < thresholdFar;
                 });

    filteredCloud->height   = 1;
    filteredCloud->is_dense = false;
    filteredCloud->header   = p_cloud_in->header;
    filteredCloud->width    = filteredCloud->size();

    return filteredCloud;
}
template pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    Utils::pointcloudDistanceFilter<pcl::PointXYZRGBA>(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
