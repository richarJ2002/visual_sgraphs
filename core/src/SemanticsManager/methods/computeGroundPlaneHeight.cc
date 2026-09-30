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

#include "SemanticsManager.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::computeGroundPlaneHeight(
    geometric::Plane     *p_groundPlane_in,
    std::optional<float> &groundPlaneHeight_out)
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
    pcl::transformPointCloud(*p_planeCloud, *p_transformedCloud, planePoseMat);

    /* Not a median: partial_sort with std::greater keeps the lower half in
       descending order, so [numPoint-1] is the upper edge of that half. */
    std::vector<float> yValues;
    for (const pcl::PointXYZRGBA &point : p_transformedCloud->points)
    {
        yValues.push_back(point.y);
    }

    size_t lowerHalfPointCount = yValues.size() / 2;

    /* An empty (or single-point) support cloud -- plane created before its
       first refit, or cleared during replaceMapClouds -- makes numPoint == 0,
       leaving nothing for [numPoint - 1] to address. Report "unknown" rather
       than substituting 0.0, which is a valid real height and would silently
       corrupt filterGroundPlanes' threshold. */
    if (lowerHalfPointCount == 0)
    {
        groundPlaneHeight_out = std::nullopt;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::partial_sort(yValues.begin(),
                      yValues.begin() + lowerHalfPointCount,
                      yValues.end(),
                      std::greater<float>());

    groundPlaneHeight_out = yValues[lowerHalfPointCount - 1];
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
