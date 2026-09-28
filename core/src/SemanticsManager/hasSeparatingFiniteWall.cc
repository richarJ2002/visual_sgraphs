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
 * @brief       Tests whether an observed finite wall separates two positions.
 *
 *              The infinite plane performs the side test, while the mapped
 *              cloud bounds reject unrelated coplanar wall segments. This is
 *              used as a veto when connected free-space evidence suggests two
 *              room hypotheses may describe the same physical room.
 *
 * @param[in]   wallList_World_in
 *              Candidate wall surfaces expressed in the active map frame.
 * @param[in]   firstPoint_World_m_in
 *              First position expressed in the active map frame, in metres.
 * @param[in]   secondPoint_World_m_in
 *              Second position expressed in the active map frame, in metres.
 * @param[in]   finiteBoundsMargin_m_in
 *              Margin applied around the observed wall-cloud bounds.
 *
 * @return      True when the segment crosses an observed finite wall patch.
 */
bool hasSeparatingFiniteWall(
    const std::vector<geometric::Plane *> &wallList_World_in,
    const Eigen::Vector3d                 &firstPoint_World_m_in,
    const Eigen::Vector3d                 &secondPoint_World_m_in,
    const double                           finiteBoundsMargin_m_in)
{
    constexpr double minimumSideDistance_m = 0.10;

    for (geometric::Plane *p_wall : wallList_World_in)
    {
        if (p_wall == nullptr || p_wall->isBad() ||
            p_wall->getPlaneType() != geometric::Plane::PlaneVariant::WALL)
        {
            continue;
        }

        const geometric::Plane::GeometrySnapshot wallGeometry =
            p_wall->getGeometrySnapshot();
        Eigen::Vector4d wallEquation_World = wallGeometry.equation_World;
        const double    wallNormalNorm = wallEquation_World.head<3>().norm();

        if (!wallEquation_World.allFinite() || wallNormalNorm < 1e-8)
        {
            continue;
        }

        wallEquation_World /= wallNormalNorm;

        const double firstSide_m =
            wallEquation_World.head<3>().dot(firstPoint_World_m_in) +
            wallEquation_World(3);
        const double secondSide_m =
            wallEquation_World.head<3>().dot(secondPoint_World_m_in) +
            wallEquation_World(3);

        if (firstSide_m * secondSide_m >= 0.0 ||
            std::abs(firstSide_m) < minimumSideDistance_m ||
            std::abs(secondSide_m) < minimumSideDistance_m)
        {
            continue;
        }

        const double interpolation = firstSide_m / (firstSide_m - secondSide_m);
        const Eigen::Vector3d intersection_World_m =
            firstPoint_World_m_in +
            interpolation * (secondPoint_World_m_in - firstPoint_World_m_in);

        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallSupportCloud =
            wallGeometry.supportCloud;

        if (p_wallSupportCloud == nullptr || p_wallSupportCloud->empty())
        {
            continue;
        }

        const Eigen::Vector3d wallCentroid_World_m =
            wallGeometry.centroid_World_m;
        const Eigen::Vector3d wallAxisU_World =
            wallEquation_World.head<3>().unitOrthogonal().normalized();
        const Eigen::Vector3d wallAxisV_World =
            wallEquation_World.head<3>().cross(wallAxisU_World).normalized();

        double minimumWallU_m = std::numeric_limits<double>::infinity();
        double maximumWallU_m = -std::numeric_limits<double>::infinity();
        double minimumWallV_m = std::numeric_limits<double>::infinity();
        double maximumWallV_m = -std::numeric_limits<double>::infinity();

        for (const pcl::PointXYZRGBA &wallPoint : p_wallSupportCloud->points)
        {
            if (!pcl::isFinite(wallPoint))
            {
                continue;
            }

            const Eigen::Vector3d wallPoint_World_m(
                static_cast<double>(wallPoint.x),
                static_cast<double>(wallPoint.y),
                static_cast<double>(wallPoint.z));
            const Eigen::Vector3d wallOffset_World_m =
                wallPoint_World_m - wallCentroid_World_m;

            const double wallU_m = wallOffset_World_m.dot(wallAxisU_World);
            const double wallV_m = wallOffset_World_m.dot(wallAxisV_World);

            minimumWallU_m = std::min(minimumWallU_m, wallU_m);
            maximumWallU_m = std::max(maximumWallU_m, wallU_m);
            minimumWallV_m = std::min(minimumWallV_m, wallV_m);
            maximumWallV_m = std::max(maximumWallV_m, wallV_m);
        }

        if (!std::isfinite(minimumWallU_m) || !std::isfinite(maximumWallU_m) ||
            !std::isfinite(minimumWallV_m) || !std::isfinite(maximumWallV_m))
        {
            continue;
        }

        const Eigen::Vector3d intersectionOffset_World_m =
            intersection_World_m - wallCentroid_World_m;
        const double intersectionU_m =
            intersectionOffset_World_m.dot(wallAxisU_World);
        const double intersectionV_m =
            intersectionOffset_World_m.dot(wallAxisV_World);

        if (intersectionU_m >= minimumWallU_m - finiteBoundsMargin_m_in &&
            intersectionU_m <= maximumWallU_m + finiteBoundsMargin_m_in &&
            intersectionV_m >= minimumWallV_m - finiteBoundsMargin_m_in &&
            intersectionV_m <= maximumWallV_m + finiteBoundsMargin_m_in)
        {
            return true;
        }
    }

    return false;
}

} // namespace core
} // namespace vs_graphs
