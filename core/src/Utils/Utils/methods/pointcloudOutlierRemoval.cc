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
 * @file            pointcloudOutlierRemoval.cc
 *
 * @brief           Implements the
 *                  Utils::pointcloudOutlierRemoval(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <pcl/filters/statistical_outlier_removal.h>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr Utils::pointcloudOutlierRemoval(
    const typename pcl::PointCloud<PointT>::Ptr &p_cloud_in,
    const int                                    meanThreshold_in,
    const float                                  stdDevThreshold_in)
{
    // Check if the input p_cloud_in is empty
    if (p_cloud_in->points.size() == 0)
        return p_cloud_in;

    // Create a container for the filtered p_cloud_in
    typename pcl::PointCloud<PointT>::Ptr p_filteredCloud(
        new pcl::PointCloud<PointT>);

    // Create the filtering object: StatisticalOutlierRemoval
    pcl::StatisticalOutlierRemoval<PointT> outlierRemoval;
    outlierRemoval.setInputCloud(p_cloud_in);
    outlierRemoval.setMeanK(meanThreshold_in);
    outlierRemoval.setStddevMulThresh(stdDevThreshold_in);
    outlierRemoval.filter(*p_filteredCloud);

    p_filteredCloud->header = p_cloud_in->header;
    p_filteredCloud->width  = p_filteredCloud->size();
    p_filteredCloud->height = 1;

    // Return the filtered p_cloud_in
    return p_filteredCloud;
}
template pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
    Utils::pointcloudOutlierRemoval<pcl::PointXYZRGBA>(
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &,
        const int,
        const float);

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
