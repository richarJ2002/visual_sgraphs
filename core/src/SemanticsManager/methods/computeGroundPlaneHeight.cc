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

namespace vs_graphs
{
namespace core
{

std::optional<float>
    SemanticsManager::computeGroundPlaneHeight(geometric::Plane *groundPlane)
{
    /* Transform the planeCloud according to the planePose */
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr planeCloud =
        groundPlane->getGeometrySnapshot().supportCloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*planeCloud, *transformedCloud, planePoseMat);

    /* Not a median: partial_sort with std::greater keeps the lower half in
       descending order, so [numPoint-1] is the upper edge of that half. */
    std::vector<float> yVals;
    for (const auto &point : transformedCloud->points)
    {
        yVals.push_back(point.y);
    }

    size_t numPoint = yVals.size() / 2;

    /* An empty (or single-point) support cloud -- plane created before its
       first refit, or cleared during replaceMapClouds -- makes numPoint == 0,
       leaving nothing for [numPoint - 1] to address. Report "unknown" rather
       than substituting 0.0, which is a valid real height and would silently
       corrupt filterGroundPlanes' threshold. */
    if (numPoint == 0)
    {
        return std::nullopt;
    }

    std::partial_sort(yVals.begin(),
                      yVals.begin() + numPoint,
                      yVals.end(),
                      std::greater<float>());

    return yVals[numPoint - 1];
}

} // namespace core
} // namespace vs_graphs
