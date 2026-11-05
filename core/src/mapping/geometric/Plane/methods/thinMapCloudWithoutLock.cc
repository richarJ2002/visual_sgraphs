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
 * @file            thinMapCloudWithoutLock.cc
 *
 * @brief           Implements Plane::thinMapCloudWithoutLock(), declared in
 *                  mapping/geometric/Plane/objects/PlaneClass.h.
 */

#include "mapping/geometric/Plane/objects/PlaneClass.h"

#include <cstdint>
#include <rclcpp/logging.hpp>

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"

namespace vs_graphs
{
namespace core
{
namespace geometric
{

PlaneStatus Plane::thinMapCloudWithoutLock(void)
{
    p_octree->deleteTree();
    if (planeCloud == nullptr || planeCloud->empty())
    {
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    /* Appends write the points directly; PCL needs an unorganized shape. */
    planeCloud->width  = static_cast<std::uint32_t>(planeCloud->size());
    planeCloud->height = 1;

    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /*!
     * One point per voxel: a voxel already holding a single accumulated
     * point must survive, so the minimum count is 1, not the segmenter's
     * per-frame noise threshold.
     */
    constexpr unsigned int MINIMUM_POINTS_PER_VOXEL = 1U;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_thinnedCloud;
    if (utils::utils::Utils::pointcloudDownsample<pcl::PointXYZRGBA>(
            planeCloud,
            p_params->refineMapPoints.octree.resolution,
            MINIMUM_POINTS_PER_VOXEL,
            p_thinnedCloud) != utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: pointcloudDownsample returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Swap only the points so header and sensor pose stay with the plane. */
    planeCloud->points.swap(p_thinnedCloud->points);
    planeCloud->width  = static_cast<std::uint32_t>(planeCloud->size());
    planeCloud->height = 1;

    p_octree->setInputCloud(planeCloud);
    p_octree->addPointsFromInputCloud();

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} /* namespace geometric */
} /* namespace core */
} /* namespace vs_graphs */
