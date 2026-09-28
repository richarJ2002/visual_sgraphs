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
 * @file            clusterPlaneClouds.cc
 *
 * @brief           Implements Utils::clusterPlaneClouds(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::clusterPlaneClouds(
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &p_cloud_in,
    std::vector<pcl::PointIndices>                &clusterIndices_inout)
{
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr p_tree(
        new pcl::search::KdTree<pcl::PointXYZRGBA>);
    p_tree->setInputCloud(p_cloud_in);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    types::SystemParams                               *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    ec.setClusterTolerance(
        p_params->seg.planeAssociation.clusterSeparation.tolerance);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(2500000);
    ec.setSearchMethod(p_tree);
    ec.setInputCloud(p_cloud_in);
    ec.extract(clusterIndices_inout);

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
