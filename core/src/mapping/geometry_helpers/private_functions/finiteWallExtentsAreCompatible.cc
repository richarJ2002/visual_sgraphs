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
 * @file            finiteWallExtentsAreCompatible.cc
 *
 * @brief           Implements finiteWallExtentsAreCompatible(), declared in
 *                  Utils/Utils/private_functions.h.
 */

#include "mapping/geometry_helpers/private_functions/private_functions.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <pcl/common/point_tests.h>
#include <rclcpp/logging.hpp>

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
    const Eigen::Vector4d                              &commonEquation_world_in,
    const double                                        observerSide_m_in,
    const std::vector<geometric::Plane *>              &mappedPlanes_in,
    const double                                        maximumInPlaneGap_m_in,
    const double minimumOrthogonalOverlap_m_in,
    const double touchingTolerance_m_in,
    bool        &areCompatible_out,
    bool        &isSeparatedByCrossingWall_out)
{
    isSeparatedByCrossingWall_out = false;

    const double commonNormalNorm = commonEquation_world_in.head<3>().norm();

    if (!commonEquation_world_in.allFinite() || commonNormalNorm < 1e-8)
    {
        areCompatible_out = false;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    const Eigen::Vector3d planeNormal_world =
        commonEquation_world_in.head<3>() / commonNormalNorm;
    const double planeOffset_m = commonEquation_world_in(3) / commonNormalNorm;

    const geometric::InPlaneAxes commonAxes =
        geometric::computeInPlaneAxes(planeNormal_world);
    const Eigen::Vector3d &tangentU_world = commonAxes.planeAxisU_world;
    const Eigen::Vector3d &tangentV_world = commonAxes.planeAxisV_world;

    ProjectedPlaneBounds firstBounds{};
    if (projectPlaneBounds(p_firstCloud_in,
                           tangentU_world,
                           tangentV_world,
                           firstBounds) != UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: projectPlaneBounds returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    ProjectedPlaneBounds secondBounds{};
    if (projectPlaneBounds(p_secondCloud_in,
                           tangentU_world,
                           tangentV_world,
                           secondBounds) != UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: projectPlaneBounds returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
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

    /*!
     * A crossing wall must reach at least this far into the observed side, in
     * metres: deeper than any wall thickness, so a door jamb or a wall behind
     * the face cannot separate two fragments, yet far shorter than a room
     * partition.
     */
    constexpr double minimumCrossingWallDepth_m = 0.50;

    /*!
     * Largest |cos| between the two normals for a wall to count as crossing
     * the face (normals at least 60 degrees apart).
     */
    constexpr double maximumCrossingNormalAlignment = 0.50;

    /*!
     * Fragments of neighbouring rooms meet at a partition or corridor wall
     * that runs away from the face on the observed side. A door gap in one
     * continuous face has no such wall, so only that case is refused here.
     * Without a known observer side the observed side is unknown: skip.
     */
    for (geometric::Plane *p_crossingWall : mappedPlanes_in)
    {
        if (observerSide_m_in == 0.0)
        {
            break;
        }

        bool crossingWallIsBad{};
        if (!(p_crossingWall == nullptr) &&
            p_crossingWall->isBad(crossingWallIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        geometric::Plane::PlaneVariant crossingWallType{};
        if (!(p_crossingWall == nullptr || crossingWallIsBad) &&
            p_crossingWall->getPlaneType(crossingWallType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_crossingWall == nullptr || crossingWallIsBad ||
            crossingWallType != geometric::Plane::PlaneVariant::WALL)
        {
            continue;
        }

        geometric::Plane::GeometrySnapshot crossingGeometry{};
        if (p_crossingWall->getGeometrySnapshot(crossingGeometry) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGeometrySnapshot returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3d crossingNormal_world =
            crossingGeometry.planeEquation_world.head<3>();
        const double crossingNormalNorm = crossingNormal_world.norm();

        if (!crossingNormal_world.allFinite() || crossingNormalNorm < 1e-8 ||
            std::abs(crossingNormal_world.dot(planeNormal_world)) >
                maximumCrossingNormalAlignment * crossingNormalNorm)
        {
            continue;
        }

        /* In-plane direction along which the crossing wall cuts the face. */
        const Eigen::Vector3d crossingAxis_world =
            (crossingNormal_world -
             crossingNormal_world.dot(planeNormal_world) * planeNormal_world)
                .normalized();
        const double axisAlongU = crossingAxis_world.dot(tangentU_world);
        const double axisAlongV = crossingAxis_world.dot(tangentV_world);

        /* Fragment ranges along the crossing axis, from their U/V boxes. */
        const std::array<const ProjectedPlaneBounds *, 2> fragmentBounds{
            &firstBounds,
            &secondBounds};
        std::array<double, 2> fragmentStart_m{};
        std::array<double, 2> fragmentEnd_m{};

        for (std::size_t fragmentIndex = 0; fragmentIndex < 2U; ++fragmentIndex)
        {
            const ProjectedPlaneBounds &bounds = *fragmentBounds[fragmentIndex];
            const double cornerMinimumU_m      = axisAlongU * bounds.minimumU_m;
            const double cornerMaximumU_m      = axisAlongU * bounds.maximumU_m;
            const double cornerMinimumV_m      = axisAlongV * bounds.minimumV_m;
            const double cornerMaximumV_m      = axisAlongV * bounds.maximumV_m;

            fragmentStart_m[fragmentIndex] =
                std::min(cornerMinimumU_m, cornerMaximumU_m) +
                std::min(cornerMinimumV_m, cornerMaximumV_m);
            fragmentEnd_m[fragmentIndex] =
                std::max(cornerMinimumU_m, cornerMaximumU_m) +
                std::max(cornerMinimumV_m, cornerMaximumV_m);
        }

        const double crossingPosition_m =
            crossingGeometry.planeCentroid_world_m.dot(crossingAxis_world);
        const bool firstBeforeCrossing =
            fragmentEnd_m[0] <= crossingPosition_m + touchingTolerance_m_in &&
            fragmentStart_m[1] >= crossingPosition_m - touchingTolerance_m_in;
        const bool secondBeforeCrossing =
            fragmentEnd_m[1] <= crossingPosition_m + touchingTolerance_m_in &&
            fragmentStart_m[0] >= crossingPosition_m - touchingTolerance_m_in;

        if (!firstBeforeCrossing && !secondBeforeCrossing)
        {
            continue;
        }

        ProjectedPlaneBounds crossingBounds{};
        if (projectPlaneBounds(crossingGeometry.supportCloud,
                               crossingAxis_world,
                               planeNormal_world,
                               crossingBounds) !=
            UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: projectPlaneBounds returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!crossingBounds.isValid)
        {
            continue;
        }

        /* Signed depths of the crossing wall, positive on the observed side. */
        const double nearDepth_m =
            observerSide_m_in > 0.0
                ? crossingBounds.minimumV_m + planeOffset_m
                : -(crossingBounds.maximumV_m + planeOffset_m);
        const double farDepth_m =
            observerSide_m_in > 0.0
                ? crossingBounds.maximumV_m + planeOffset_m
                : -(crossingBounds.minimumV_m + planeOffset_m);

        if (nearDepth_m <= touchingTolerance_m_in &&
            farDepth_m >= minimumCrossingWallDepth_m)
        {
            areCompatible_out             = false;
            isSeparatedByCrossingWall_out = true;
            return UtilsStatus::UTILS_STATUS_SUCCESS;
        }
    }

    const double gapU_m = std::max(0.0, -overlapU_m);
    const double gapV_m = std::max(0.0, -overlapV_m);

    areCompatible_out = (gapU_m <= maximumInPlaneGap_m_in &&
                         overlapV_m >= minimumOrthogonalOverlap_m_in) ||
                        (gapV_m <= maximumInPlaneGap_m_in &&
                         overlapU_m >= minimumOrthogonalOverlap_m_in);
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
