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
 * @file            applyTransform.cc
 *
 * @brief           Implements Plane::applyTransform(), declared in
 *                  Geometric/Plane.h.
 */

#include "Geometric/Plane.h"
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

PlaneStatus
    Plane::applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in)
{
    std::scoped_lock lock(positionMutex, typeMutex, featuresMutex);

    centroid = transform_oldWorldToNewWorld_in.map(centroid);

    for (pcl::PointXYZRGBA &point : planeCloud->points)
    {
        Eigen::Vector3d pointVector(point.x, point.y, point.z);

        pointVector = transform_oldWorldToNewWorld_in.map(pointVector);

        point.x = static_cast<float>(pointVector.x());
        point.y = static_cast<float>(pointVector.y());
        point.z = static_cast<float>(pointVector.z());
    }

    g2o::Plane3D transformedEquation{};
    if (transformPlaneEquation(globalEquation,
                               transform_oldWorldToNewWorld_in,
                               transformedEquation) !=
        PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: transformPlaneEquation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    globalEquation = transformedEquation;

    p_octree->deleteTree();
    p_octree->setInputCloud(planeCloud);
    p_octree->addPointsFromInputCloud();

    /* Recompute the finite bounds in the transformed frame. */
    if (updatePlaneBoundsWithoutLock() != PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updatePlaneBoundsWithoutLock returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    ++cloudGeneration;
    successfulRefitGeneration = cloudGeneration;

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
