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
 * @file            getMapClouds.cc
 *
 * @brief           Implements Plane::getMapClouds(), declared in
 *                  Geometric/Plane.h.
 */

#include "Geometric/Plane.h"
#include <algorithm>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <limits>
#include <pcl/octree/octree_search.h>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

PlaneStatus Plane::getMapClouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &mapClouds_out)
{
    /*!
     * Publishers retain this result after the lock is released, so return a
     * snapshot instead of an alias to the concurrently updated member cloud.
     */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_planeCloudCopy(
        new pcl::PointCloud<pcl::PointXYZRGBA>);

    std::scoped_lock lock(positionMutex, featuresMutex);
    if (planeCloud != nullptr)
    {
        *p_planeCloudCopy = *planeCloud;
    }

    mapClouds_out = p_planeCloudCopy;
    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
