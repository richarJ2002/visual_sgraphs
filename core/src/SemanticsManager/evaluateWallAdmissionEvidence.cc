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
 * @file            evaluateWallAdmissionEvidence.cc
 *
 * @brief           Implements evaluateWallAdmissionEvidence(), declared in
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

SemanticsManagerStatus
    evaluateWallAdmissionEvidence(geometric::Plane          *p_wall_in,
                                  const types::SystemParams *p_systemParams_in,
                                  const Eigen::Vector3d &groundNormal_world_in,
                                  WallAdmissionEvidence &admissionEvidence_out)
{
    WallAdmissionEvidence evidence;

    bool wallIsBad{};
    if (!(p_wall_in == nullptr) &&
        p_wall_in->isBad(wallIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_wall_in == nullptr || wallIsBad || p_systemParams_in == nullptr)
    {
        admissionEvidence_out = evidence;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::size_t wallGetObservationCount{};
    if (p_wall_in->getObservationCount(wallGetObservationCount) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservationCount returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    evidence.observationCount = wallGetObservationCount;

    geometric::Plane::GeometrySnapshot geometry{};
    if (p_wall_in->getGeometrySnapshot(geometry) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometrySnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d equation_world = geometry.equation_world;
    const double    normalNorm     = equation_world.head<3>().norm();

    if (!equation_world.allFinite() || !std::isfinite(normalNorm) ||
        normalNorm < 1e-8)
    {
        admissionEvidence_out = evidence;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    equation_world /= normalNorm;
    const Eigen::Vector3d normal_world = equation_world.head<3>();

    if (!equation_world.allFinite() ||
        std::abs(normal_world.norm() - 1.0) > 1e-6)
    {
        admissionEvidence_out = evidence;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_supportCloud =
        geometry.supportCloud;

    if (p_supportCloud == nullptr)
    {
        admissionEvidence_out = evidence;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /*!
     * Ground-aligned in-plane axes so the two extents below correspond to
     * physical horizontal width and vertical height, not an arbitrary
     * in-plane rotation. A door-frame post rotated relative to an arbitrary
     * unitOrthogonal() axis can inflate BOTH bounding-box extents past a
     * width/height threshold even though its true width is a few
     * centimetres -- ground-anchoring the axes removes that degree of
     * freedom. axisHorizontal is the wall's own horizontal (along-wall)
     * direction: orthogonal to both the ground normal and the wall normal.
     * Falls back to the previous arbitrary-orthogonal axes when no ground
     * plane is available yet (early in a mission) or the wall is itself
     * near-horizontal (groundNormal parallel to normal_world).
     */
    const double    groundNormalNorm = groundNormal_world_in.norm();
    Eigen::Vector3d axisU_world      = Eigen::Vector3d::Zero();
    Eigen::Vector3d axisV_world      = Eigen::Vector3d::Zero();
    if (std::isfinite(groundNormalNorm) && groundNormalNorm > 1e-8)
    {
        const Eigen::Vector3d unitGroundNormal_world =
            groundNormal_world_in / groundNormalNorm;
        const Eigen::Vector3d horizontalCandidate_world =
            unitGroundNormal_world.cross(normal_world);
        const double horizontalNorm = horizontalCandidate_world.norm();
        if (std::isfinite(horizontalNorm) && horizontalNorm > 1e-3)
        {
            axisU_world = horizontalCandidate_world / horizontalNorm;
            axisV_world = axisU_world.cross(normal_world).normalized();
        }
    }
    if (axisU_world.squaredNorm() < 0.5 || axisV_world.squaredNorm() < 0.5)
    {
        axisU_world = normal_world.unitOrthogonal().normalized();
        axisV_world = normal_world.cross(axisU_world).normalized();
    }
    double minimumU_m = std::numeric_limits<double>::infinity();
    double maximumU_m = -std::numeric_limits<double>::infinity();
    double minimumV_m = std::numeric_limits<double>::infinity();
    double maximumV_m = -std::numeric_limits<double>::infinity();

    for (const pcl::PointXYZRGBA &point : p_supportCloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        const Eigen::Vector3d point_world_m(point.x, point.y, point.z);
        evidence.finitePointCount++;

        const double fitDistance_m =
            std::abs(normal_world.dot(point_world_m) + equation_world(3));

        if (fitDistance_m > p_systemParams_in->seg.ransac.distanceThresh)
        {
            continue;
        }

        const double pointU_m = point_world_m.dot(axisU_world);
        const double pointV_m = point_world_m.dot(axisV_world);
        minimumU_m            = std::min(minimumU_m, pointU_m);
        maximumU_m            = std::max(maximumU_m, pointU_m);
        minimumV_m            = std::min(minimumV_m, pointV_m);
        maximumV_m            = std::max(maximumV_m, pointV_m);
        evidence.fittedPointCount++;
    }

    const double extentU_m     = maximumU_m - minimumU_m;
    const double extentV_m     = maximumV_m - minimumV_m;
    const double majorExtent_m = std::max(extentU_m, extentV_m);
    const double minorExtent_m = std::min(extentU_m, extentV_m);
    const double area_m2       = majorExtent_m * minorExtent_m;
    const types::SystemParams::SemSeg::WallCreation &wallCreation =
        p_systemParams_in->semSeg.wallCreation;
    const double fitSupportRatio =
        evidence.finitePointCount > 0U
            ? static_cast<double>(evidence.fittedPointCount) /
                  static_cast<double>(evidence.finitePointCount)
            : 0.0;

    evidence.hasAdequateFiniteFit =
        evidence.fittedPointCount >= 20U &&
        fitSupportRatio >= p_systemParams_in->roomSeg.minimumWallSupportRatio &&
        std::isfinite(majorExtent_m) && std::isfinite(minorExtent_m) &&
        std::isfinite(area_m2) &&
        majorExtent_m >= wallCreation.minimumMajorExtent_m &&
        minorExtent_m >= wallCreation.minimumMinorExtent_m &&
        area_m2 >= wallCreation.minimumArea_m2;

    const std::size_t strongObservationPointCount =
        wallCreation.connectivity.enabled
            ? std::max<std::size_t>(
                  wallCreation.minimumPointCount,
                  wallCreation.connectivity.minimumComponentPointCount)
            : wallCreation.minimumPointCount;
    const bool strongFirstObservationEvidence =
        evidence.hasAdequateFiniteFit &&
        evidence.fittedPointCount >= strongObservationPointCount;
    const bool repeatedObservationEvidence =
        evidence.observationCount >=
        std::max<std::size_t>(
            p_systemParams_in->roomSeg.minimumWallObservationCount,
            1U);
    geometric::Plane::PlaneVariant wallPlaneType{};
    if (p_wall_in->getPlaneType(wallPlaneType) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPlaneType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    geometric::Plane::PlaneVariant wallExpectedPlaneType{};
    if ((wallPlaneType == geometric::Plane::PlaneVariant::WALL) &&
        p_wall_in->getExpectedPlaneType(wallExpectedPlaneType) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getExpectedPlaneType returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const bool wallDominatesSemantics =
        wallPlaneType == geometric::Plane::PlaneVariant::WALL &&
        wallExpectedPlaneType == geometric::Plane::PlaneVariant::WALL;

    evidence.isAdmissible =
        wallDominatesSemantics && evidence.hasAdequateFiniteFit &&
        (repeatedObservationEvidence || strongFirstObservationEvidence);
    admissionEvidence_out = evidence;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
