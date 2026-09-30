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
 * @file            updatePlaneBoundsWithoutLock.cc
 *
 * @brief           Implements Plane::updatePlaneBoundsWithoutLock(), declared
 *                  in Geometric/Plane.h.
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

PlaneStatus Plane::updatePlaneBoundsWithoutLock(void)
{

    /* Reset the plane size */
    minPlaneU = std::numeric_limits<double>::max();
    maxPlaneU = std::numeric_limits<double>::lowest();
    minPlaneV = std::numeric_limits<double>::max();
    maxPlaneV = std::numeric_limits<double>::lowest();

    /* Extract the coefficient of the wall */
    Eigen::Vector4d wallEquation = globalEquation.coeffs();

    /* Extract the magnitude of the norm from the coefficients */
    const double normalMagnitude = wallEquation.head<3>().norm();

    if (!std::isfinite(normalMagnitude) || normalMagnitude < 1e-8)
    {
        std::cerr << "[GeoSemHelper] Cannot create open passage: wall " << id
                  << " has an invalid plane equation." << std::endl;
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    /* Normalize the norm vector */
    Eigen::Vector3d normalVector = wallEquation.head<3>() / normalMagnitude;

    for (Eigen::Index componentIndex = 0; componentIndex < normalVector.size();
         ++componentIndex)
    {
        if (std::abs(normalVector(componentIndex)) <= 1e-12)
        {
            continue;
        }
        if (normalVector(componentIndex) < 0.0)
        {
            normalVector = -normalVector;
        }
        break;
    }

    /* Ensure the plane normal is valid */
    if (!normalVector.allFinite() ||
        normalVector.squaredNorm() < std::numeric_limits<double>::epsilon())
    {
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    /* Normalize the plane normal in the world frame */
    const Eigen::Vector3d planeNormal = normalVector.normalized();

    /* Match finiteWallExtentsAreCompatible(): deterministic X/Y reference. */
    const Eigen::Vector3d referenceAxis = std::abs(planeNormal.x()) <= 0.90
                                              ? Eigen::Vector3d::UnitX()
                                              : Eigen::Vector3d::UnitY();

    /* Construct orthonormal axes lying inside the plane */
    const Eigen::Vector3d axisU = planeNormal.cross(referenceAxis).normalized();
    const Eigen::Vector3d axisV = planeNormal.cross(axisU).normalized();

    /* Init flag to indicate that a valid point was found */
    bool foundValidPoint = false;

    /* Skip if cloud is invalid */
    if (!planeCloud || planeCloud->empty())
    {
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    /* Collect in-plane projections first: bounds are outlier-trimmed
     * percentiles, not raw minima/maxima, so a few stray map points (bad
     * triangulations, edge bleed from neighbouring surfaces) cannot
     * stretch a wall quad metres past its real surface. Sparse clouds keep
     * exact min/max -- trimming needs enough samples to mean anything. */
    std::vector<double> projectionsU;
    std::vector<double> projectionsV;
    projectionsU.reserve(planeCloud->points.size());
    projectionsV.reserve(planeCloud->points.size());

    /* Iterate through associated map points */
    for (const pcl::PointXYZRGBA &point : planeCloud->points)
    {
        /* Skip points with invalid coordinates */
        if (!pcl::isFinite(point))
        {
            continue;
        }

        /* Convert PCL to Eigen */
        const Eigen::Vector3d point_World(static_cast<double>(point.x),
                                          static_cast<double>(point.y),
                                          static_cast<double>(point.z));

        /* Emit actual canonical world projections, not centroid-relative
         * extents. */
        projectionsU.push_back(point_World.dot(axisU));
        projectionsV.push_back(point_World.dot(axisV));

        /* Set flag to indicate that a valid point is linked with the plane */
        foundValidPoint = true;
    }

    /* Reset to zero when no valid supporting points exist */
    if (!foundValidPoint)
    {
        isFlaggedBad = true;
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    constexpr std::size_t minimumRobustPoints = 10U;
    constexpr double      lowerPercentile     = 0.01;
    constexpr double      upperPercentile     = 0.99;

    const auto robustBound =
        [](std::vector<double> &samples_inout, double percentile_in)
    {
        std::sort(samples_inout.begin(), samples_inout.end());
        const double sampleIndex =
            percentile_in * static_cast<double>(samples_inout.size() - 1U);
        return samples_inout[static_cast<std::size_t>(sampleIndex)];
    };

    if (projectionsU.size() < minimumRobustPoints)
    {
        const auto [minU, maxU] =
            std::minmax_element(projectionsU.begin(), projectionsU.end());
        const auto [minV, maxV] =
            std::minmax_element(projectionsV.begin(), projectionsV.end());
        minPlaneU = *minU;
        maxPlaneU = *maxU;
        minPlaneV = *minV;
        maxPlaneV = *maxV;
    }
    else
    {
        minPlaneU = robustBound(projectionsU, lowerPercentile);
        maxPlaneU = robustBound(projectionsU, upperPercentile);
        minPlaneV = robustBound(projectionsV, lowerPercentile);
        maxPlaneV = robustBound(projectionsV, upperPercentile);
    }

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
