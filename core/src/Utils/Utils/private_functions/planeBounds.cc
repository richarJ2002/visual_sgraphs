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
 * @file            planeBounds.cc
 *
 * @brief           Implements projectPlaneBounds() and
 *                  finiteWallExtentsAreCompatible(), declared in
 *                  Utils/Utils/private_functions.h.
 */

#include "Utils/Utils/private_functions.h"

#include <algorithm>
#include <cmath>

#include <pcl/common/point_tests.h>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

ProjectedPlaneBounds projectPlaneBounds(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_planeCloud_in,
    const Eigen::Vector3d                              &tangentU_World_in,
    const Eigen::Vector3d                              &tangentV_World_in)
{
    ProjectedPlaneBounds bounds;

    if (p_planeCloud_in == nullptr)
    {
        return bounds;
    }

    for (const pcl::PointXYZRGBA &point_World_m : p_planeCloud_in->points)
    {
        if (!pcl::isFinite(point_World_m))
        {
            continue;
        }

        const Eigen::Vector3d position_World_m(point_World_m.x,
                                               point_World_m.y,
                                               point_World_m.z);

        const double coordinateU_m = tangentU_World_in.dot(position_World_m);
        const double coordinateV_m = tangentV_World_in.dot(position_World_m);

        bounds.minimumU_m = std::min(bounds.minimumU_m, coordinateU_m);
        bounds.maximumU_m = std::max(bounds.maximumU_m, coordinateU_m);
        bounds.minimumV_m = std::min(bounds.minimumV_m, coordinateV_m);
        bounds.maximumV_m = std::max(bounds.maximumV_m, coordinateV_m);
        bounds.valid      = true;
    }

    return bounds;
}

bool finiteWallExtentsAreCompatible(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_firstCloud_in,
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_secondCloud_in,
    const Eigen::Vector3d                              &commonNormal_World_in,
    const double                                        maximumInPlaneGap_m_in,
    const double minimumOrthogonalOverlap_m_in)
{
    if (!commonNormal_World_in.allFinite() ||
        commonNormal_World_in.norm() < 1e-8)
    {
        return false;
    }

    const Eigen::Vector3d normal_World = commonNormal_World_in.normalized();

    Eigen::Vector3d referenceAxis_World = Eigen::Vector3d::UnitX();

    if (std::abs(normal_World.dot(referenceAxis_World)) > 0.90)
    {
        referenceAxis_World = Eigen::Vector3d::UnitY();
    }

    const Eigen::Vector3d tangentU_World =
        normal_World.cross(referenceAxis_World).normalized();
    const Eigen::Vector3d tangentV_World =
        normal_World.cross(tangentU_World).normalized();

    const ProjectedPlaneBounds firstBounds =
        projectPlaneBounds(p_firstCloud_in, tangentU_World, tangentV_World);
    const ProjectedPlaneBounds secondBounds =
        projectPlaneBounds(p_secondCloud_in, tangentU_World, tangentV_World);

    if (!firstBounds.valid || !secondBounds.valid)
    {
        return false;
    }

    const double overlapU_m =
        std::min(firstBounds.maximumU_m, secondBounds.maximumU_m) -
        std::max(firstBounds.minimumU_m, secondBounds.minimumU_m);
    const double overlapV_m =
        std::min(firstBounds.maximumV_m, secondBounds.maximumV_m) -
        std::max(firstBounds.minimumV_m, secondBounds.minimumV_m);

    if ((overlapU_m >= minimumOrthogonalOverlap_m_in && overlapV_m >= 0.0) ||
        (overlapV_m >= minimumOrthogonalOverlap_m_in && overlapU_m >= 0.0))
    {
        return true;
    }

    const double gapU_m = std::max(0.0, -overlapU_m);
    const double gapV_m = std::max(0.0, -overlapV_m);

    return (gapU_m <= maximumInPlaneGap_m_in &&
            overlapV_m >= minimumOrthogonalOverlap_m_in) ||
           (gapV_m <= maximumInPlaneGap_m_in &&
            overlapU_m >= minimumOrthogonalOverlap_m_in);
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
