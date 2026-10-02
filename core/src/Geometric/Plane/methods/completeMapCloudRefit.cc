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
 * @file            completeMapCloudRefit.cc
 *
 * @brief           Implements Plane::completeMapCloudRefit().
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
    Plane::completeMapCloudRefit(const std::uint64_t sourceCloudGeneration_in,
                                 const Eigen::Vector3d &centroid_world_m_in,
                                 const g2o::Plane3D    &equation_world_in,
                                 const std::size_t      finitePointCount_in,
                                 bool                  &wasRefitPublished_out)
{
    std::scoped_lock lock(positionMutex, typeMutex, featuresMutex);

    if (sourceCloudGeneration_in != cloudGeneration)
    {
        wasRefitPublished_out = false;
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    centroid                            = centroid_world_m_in;
    globalEquation                      = equation_world_in;
    lastSuccessfulRefitFinitePointCount = finitePointCount_in;
    successfulRefitGeneration           = sourceCloudGeneration_in;

    if (updatePlaneBoundsWithoutLock() != PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updatePlaneBoundsWithoutLock returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    wasRefitPublished_out = true;
    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
