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
 * @file            setMapClouds.cc
 *
 * @brief           Implements Plane::setMapClouds(), declared in
 *                  Geometric/Plane.h.
 */

#include "mapping/geometric/Plane/objects/PlaneClass.h"
#include <algorithm>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <limits>
#include <pcl/octree/octree_search.h>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

PlaneStatus Plane::setMapClouds(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_additionalCloud_in)
{
    if (!p_additionalCloud_in || p_additionalCloud_in->empty())
    {
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    std::lock_guard<std::mutex> lock(featuresMutex);

    /* Add the new points to the plane cloud */
    for (const pcl::PointXYZRGBA &point : p_additionalCloud_in->points)
    {
        planeCloud->points.push_back(point);
    }

    /* Thinning also resets the organization metadata and the octree. */
    if (thinMapCloudWithoutLock() != PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: thinMapCloudWithoutLock returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    ++cloudGeneration;

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} /* namespace geometric */
} /* namespace core */
} /* namespace vs_graphs */
