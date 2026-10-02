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
 * @file            hasSeparatingFiniteWall.cc
 *
 * @brief           Implements hasSeparatingFiniteWall(), declared in
 *                  SemanticsManager/private_functions.h.
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

SemanticsManagerStatus hasSeparatingFiniteWall(
    const std::vector<geometric::Plane *> &wallList_world_in,
    const Eigen::Vector3d                 &firstPoint_world_m_in,
    const Eigen::Vector3d                 &secondPoint_world_m_in,
    const double                           finiteBoundsMargin_m_in,
    bool                                  &hasSeparatingFiniteWall_out)
{
    constexpr double minimumSideDistance_m = 0.10;

    for (geometric::Plane *p_wall : wallList_world_in)
    {
        bool wallIsBad{};
        if (!(p_wall == nullptr) &&
            p_wall->isBad(wallIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        geometric::Plane::PlaneVariant wallPlaneType{};
        if (!(p_wall == nullptr || wallIsBad) &&
            p_wall->getPlaneType(wallPlaneType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_wall == nullptr || wallIsBad ||
            wallPlaneType != geometric::Plane::PlaneVariant::WALL)
        {
            continue;
        }

        geometric::Plane::GeometrySnapshot wallGeometry{};
        if (p_wall->getGeometrySnapshot(wallGeometry) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGeometrySnapshot returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d wallEquation_world = wallGeometry.equation_world;
        const double    wallNormalNorm = wallEquation_world.head<3>().norm();

        if (!wallEquation_world.allFinite() || wallNormalNorm < 1e-8)
        {
            continue;
        }

        wallEquation_world /= wallNormalNorm;

        const double firstSide_m =
            wallEquation_world.head<3>().dot(firstPoint_world_m_in) +
            wallEquation_world(3);
        const double secondSide_m =
            wallEquation_world.head<3>().dot(secondPoint_world_m_in) +
            wallEquation_world(3);

        if (firstSide_m * secondSide_m >= 0.0 ||
            std::abs(firstSide_m) < minimumSideDistance_m ||
            std::abs(secondSide_m) < minimumSideDistance_m)
        {
            continue;
        }

        const double interpolation = firstSide_m / (firstSide_m - secondSide_m);
        const Eigen::Vector3d intersection_world_m =
            firstPoint_world_m_in +
            interpolation * (secondPoint_world_m_in - firstPoint_world_m_in);

        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallSupportCloud =
            wallGeometry.supportCloud;

        if (p_wallSupportCloud == nullptr || p_wallSupportCloud->empty())
        {
            continue;
        }

        const Eigen::Vector3d wallCentroid_world_m =
            wallGeometry.centroid_world_m;
        const Eigen::Vector3d wallAxisU_world =
            wallEquation_world.head<3>().unitOrthogonal().normalized();
        const Eigen::Vector3d wallAxisV_world =
            wallEquation_world.head<3>().cross(wallAxisU_world).normalized();

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

            const Eigen::Vector3d wallPoint_world_m(
                static_cast<double>(wallPoint.x),
                static_cast<double>(wallPoint.y),
                static_cast<double>(wallPoint.z));
            const Eigen::Vector3d wallOffset_world_m =
                wallPoint_world_m - wallCentroid_world_m;

            const double wallU_m = wallOffset_world_m.dot(wallAxisU_world);
            const double wallV_m = wallOffset_world_m.dot(wallAxisV_world);

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

        const Eigen::Vector3d intersectionOffset_world_m =
            intersection_world_m - wallCentroid_world_m;
        const double intersectionU_m =
            intersectionOffset_world_m.dot(wallAxisU_world);
        const double intersectionV_m =
            intersectionOffset_world_m.dot(wallAxisV_world);

        if (intersectionU_m >= minimumWallU_m - finiteBoundsMargin_m_in &&
            intersectionU_m <= maximumWallU_m + finiteBoundsMargin_m_in &&
            intersectionV_m >= minimumWallV_m - finiteBoundsMargin_m_in &&
            intersectionV_m <= maximumWallV_m + finiteBoundsMargin_m_in)
        {
            hasSeparatingFiniteWall_out = true;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }
    }

    hasSeparatingFiniteWall_out = false;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
