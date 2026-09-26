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

#include "private_functions.h"

#include <algorithm>
#include <cmath>
#include <limits>

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
bool projectPlaneFootprintOntoSharedAxes(geometric::Plane      *p_plane_in,
                                         const Eigen::Vector3d &axisU_World_in,
                                         const Eigen::Vector3d &axisV_World_in,
                                         double                &minU_m_out,
                                         double                &maxU_m_out,
                                         double                &minV_m_out,
                                         double                &maxV_m_out)
{
    if (p_plane_in == nullptr)
    {
        return false;
    }

    const geometric::Plane::GeometrySnapshot geometry =
        p_plane_in->getGeometrySnapshot();
    if (geometry.supportCloud == nullptr || geometry.supportCloud->empty())
    {
        return false;
    }

    minU_m_out = std::numeric_limits<double>::infinity();
    maxU_m_out = -std::numeric_limits<double>::infinity();
    minV_m_out = std::numeric_limits<double>::infinity();
    maxV_m_out = -std::numeric_limits<double>::infinity();

    for (const pcl::PointXYZRGBA &point : geometry.supportCloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }
        const Eigen::Vector3d point_World_m(point.x, point.y, point.z);
        const double          pointU_m = point_World_m.dot(axisU_World_in);
        const double          pointV_m = point_World_m.dot(axisV_World_in);
        minU_m_out                     = std::min(minU_m_out, pointU_m);
        maxU_m_out                     = std::max(maxU_m_out, pointU_m);
        minV_m_out                     = std::min(minV_m_out, pointV_m);
        maxV_m_out                     = std::max(maxV_m_out, pointV_m);
    }

    return std::isfinite(minU_m_out) && std::isfinite(maxU_m_out) &&
           std::isfinite(minV_m_out) && std::isfinite(maxV_m_out);
}

} // namespace core
} // namespace vs_graphs
