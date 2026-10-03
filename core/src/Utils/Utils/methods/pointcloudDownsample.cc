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
 * @file            pointcloudDownsample.cc
 *
 * @brief           Implements the
 *                  Utils::pointcloudDownsample(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <pcl/filters/voxel_grid.h>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

template <typename PointT>
UtilsStatus Utils::pointcloudDownsample(
    const typename pcl::PointCloud<PointT>::Ptr &p_cloud_in,
    const float                                  leafSize_in,
    const unsigned int                           minPointsPerVoxel_in,
    typename pcl::PointCloud<PointT>::Ptr       &p_downsampledCloud_out)
{
    // The filtered point p_cloud_in object
    typename pcl::PointCloud<PointT>::Ptr p_filteredCloud(
        new pcl::PointCloud<PointT>());

    // Define the downsampling filter
    typename pcl::VoxelGrid<PointT>::Ptr p_downsampleFilter(
        new pcl::VoxelGrid<PointT>());

    // Set the parameters of the downsampling filter
    p_downsampleFilter->setLeafSize(leafSize_in, leafSize_in, leafSize_in);
    p_downsampleFilter->setMinimumPointsNumberPerVoxel(minPointsPerVoxel_in);
    p_downsampleFilter->setInputCloud(p_cloud_in);

    // Apply the downsampling filter
    p_downsampleFilter->filter(*p_filteredCloud);
    p_filteredCloud->header = p_cloud_in->header;
    p_filteredCloud->width  = static_cast<uint32_t>(p_filteredCloud->size());
    p_filteredCloud->height = 1;

    p_downsampledCloud_out = p_filteredCloud;
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}
/*!
 * @brief           Explicit instantiation of Utils::pointcloudDownsample for
 *                  pcl::PointXYZRGBA clouds, the only point type the pipeline
 *                  uses.
 */
template UtilsStatus Utils::pointcloudDownsample<pcl::PointXYZRGBA>(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &,
    const float,
    const unsigned int,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
