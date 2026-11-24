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
 * @file            computeGroundPlaneHeight.cc
 *
 * @brief           Implements SemanticsManager::computeGroundPlaneHeight(),
 *                  declared in SemanticsManager.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::computeGroundPlaneHeight(
    geometric::Plane      *p_groundPlane_in,
    const Eigen::Vector3d &upDirection_world_in,
    std::optional<float>  &groundPlaneHeight_down_m_out)
{
    /* Transform the planeCloud according to the planePose */
    geometric::Plane::GeometrySnapshot groundPlaneGetGeometrySnapshot{};
    if (p_groundPlane_in->getGeometrySnapshot(groundPlaneGetGeometrySnapshot) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometrySnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_planeCloud =
        groundPlaneGetGeometrySnapshot.supportCloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_transformedCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*p_planeCloud,
                             *p_transformedCloud,
                             groundPlanToHorizontalTransformation);

    /* The transform turns the ground normal onto up, so a point's height
       along down is minus its part along up (its y when up is the
       map-start camera's -Y). Not a median: partial_sort with std::greater
       keeps the lower half in descending order, so [numPoint-1] is the upper
       edge of that half. */
    const Eigen::Vector3f upDirection_horizontal =
        upDirection_world_in.cast<float>();
    std::vector<float> heightValues_down_m;
    for (const pcl::PointXYZRGBA &cloudPoint : p_transformedCloud->points)
    {
        heightValues_down_m.push_back(
            -upDirection_horizontal.dot(cloudPoint.getVector3fMap()));
    }

    size_t lowerHalfPointCount = heightValues_down_m.size() / 2;

    /* An empty (or single-point) support cloud -- plane created before its
       first refit, or cleared during replaceMapClouds -- makes numPoint == 0,
       leaving nothing for [numPoint - 1] to address. Report "unknown" rather
       than substituting 0.0, which is a valid real height and would silently
       corrupt filterGroundPlanes' threshold. */
    if (lowerHalfPointCount == 0)
    {
        groundPlaneHeight_down_m_out = std::nullopt;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::partial_sort(heightValues_down_m.begin(),
                      heightValues_down_m.begin() + lowerHalfPointCount,
                      heightValues_down_m.end(),
                      std::greater<float>());

    groundPlaneHeight_down_m_out = heightValues_down_m[lowerHalfPointCount - 1];
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
