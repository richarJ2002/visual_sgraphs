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

UtilsStatus finiteWallExtentsAreCompatible(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_firstCloud_in,
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_secondCloud_in,
    const Eigen::Vector3d                              &commonNormal_World_in,
    const double                                        maximumInPlaneGap_m_in,
    const double minimumOrthogonalOverlap_m_in,
    bool        &areCompatible_out)
{
    if (!commonNormal_World_in.allFinite() ||
        commonNormal_World_in.norm() < 1e-8)
    {
        areCompatible_out = false;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
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

    ProjectedPlaneBounds firstBounds{};
    if (projectPlaneBounds(p_firstCloud_in,
                           tangentU_World,
                           tangentV_World,
                           firstBounds) != UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        // projectPlaneBounds cannot fail; continue as before.
    }
    ProjectedPlaneBounds secondBounds{};
    if (projectPlaneBounds(p_secondCloud_in,
                           tangentU_World,
                           tangentV_World,
                           secondBounds) != UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        // projectPlaneBounds cannot fail; continue as before.
    }

    if (!firstBounds.isValid || !secondBounds.isValid)
    {
        areCompatible_out = false;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
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
        areCompatible_out = true;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    const double gapU_m = std::max(0.0, -overlapU_m);
    const double gapV_m = std::max(0.0, -overlapV_m);

    areCompatible_out = (gapU_m <= maximumInPlaneGap_m_in &&
                         overlapV_m >= minimumOrthogonalOverlap_m_in) ||
                        (gapV_m <= maximumInPlaneGap_m_in &&
                         overlapU_m >= minimumOrthogonalOverlap_m_in);
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
