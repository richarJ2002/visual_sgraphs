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
 * @file            computePlaneWidthHeight.cc
 *
 * @brief           Implements Utils::computePlaneWidthHeight(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <utility>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

std::pair<double, double> Utils::computePlaneWidthHeight(
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_cloud_in)
{
    if (p_cloud_in->points.empty())
        return std::make_pair(0.0, 0.0);

    // Apply PCA to find the principal axes of the point p_cloud_in
    pcl::PCA<pcl::PointXYZRGBA> pca;
    pca.setInputCloud(p_cloud_in);

    // Project all points onto the PCA space
    pcl::PointCloud<pcl::PointXYZRGBA> projected;
    pca.project(*p_cloud_in, projected);

    // Find min/max along each principal axis
    pcl::PointXYZRGBA minPt, maxPt;
    pcl::getMinMax3D(projected, minPt, maxPt);

    // Axis 0 = largest variance (width), Axis 1 = second (height)
    double width  = static_cast<double>(maxPt.y - minPt.y);
    double height = static_cast<double>(maxPt.x - minPt.x);

    return std::make_pair(width, height);
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
