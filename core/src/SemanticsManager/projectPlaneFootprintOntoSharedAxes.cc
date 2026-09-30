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
 * @file            projectPlaneFootprintOntoSharedAxes.cc
 *
 * @brief           Implements projectPlaneFootprintOntoSharedAxes(), declared
 *                  in SemanticsManager/private_functions.h.
 */

#include "SemanticsManager.h"

#include "private_functions.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief Projects a WALL Plane's finite support cloud onto a shared in-plane
 *        tangent frame, returning the resulting axis-aligned interval.
 *
 * @return false when the plane has no usable geometry (null/empty cloud, or
 *         a degenerate equation); the caller must treat that as "cannot
 *         claim overlap" rather than as a zero-size interval.
 */
SemanticsManagerStatus
    projectPlaneFootprintOntoSharedAxes(geometric::Plane      *p_plane_in,
                                        const Eigen::Vector3d &axisU_World_in,
                                        const Eigen::Vector3d &axisV_World_in,
                                        double                &minimumU_m_out,
                                        double                &maximumU_m_out,
                                        double                &minimumV_m_out,
                                        double                &maximumV_m_out,
                                        bool                  &isProjected_out)
{
    if (p_plane_in == nullptr)
    {
        isProjected_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    geometric::Plane::GeometrySnapshot geometry{};
    if (p_plane_in->getGeometrySnapshot(geometry) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometrySnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (geometry.supportCloud == nullptr || geometry.supportCloud->empty())
    {
        isProjected_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    minimumU_m_out = std::numeric_limits<double>::infinity();
    maximumU_m_out = -std::numeric_limits<double>::infinity();
    minimumV_m_out = std::numeric_limits<double>::infinity();
    maximumV_m_out = -std::numeric_limits<double>::infinity();

    for (const pcl::PointXYZRGBA &point : geometry.supportCloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }
        const Eigen::Vector3d point_World_m(point.x, point.y, point.z);
        const double          pointU_m = point_World_m.dot(axisU_World_in);
        const double          pointV_m = point_World_m.dot(axisV_World_in);
        minimumU_m_out                 = std::min(minimumU_m_out, pointU_m);
        maximumU_m_out                 = std::max(maximumU_m_out, pointU_m);
        minimumV_m_out                 = std::min(minimumV_m_out, pointV_m);
        maximumV_m_out                 = std::max(maximumV_m_out, pointV_m);
    }

    isProjected_out =
        std::isfinite(minimumU_m_out) && std::isfinite(maximumU_m_out) &&
        std::isfinite(minimumV_m_out) && std::isfinite(maximumV_m_out);
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
