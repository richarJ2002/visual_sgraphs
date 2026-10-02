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
 * @file            buildFiniteWallSegment2d.cc
 *
 * @brief           Implements buildFiniteWallSegment2d(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "SemanticsManager.h"

#include "private_functions.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Builds a robust finite wall segment on the horizontal ground
 *                  plane.
 *
 * @param[in]       p_wall_in
 *                  Wall whose observed cloud defines the finite extent.
 *
 * @param[in]       groundNormal_world_in
 *                  Unit ground normal in the world frame.
 *
 * @param[in]       groundAxisU_world_in
 *                  First horizontal ground axis.
 *
 * @param[in]       groundAxisV_world_in
 *                  Second horizontal ground axis.
 *
 * @param[in]       endpointTrimRatio_in
 *                  Fraction trimmed from both extent tails.
 *
 * @param[in]       minimumWallLength_m_in
 *                  Minimum accepted horizontal length.
 *
 * @param[in,out]   segment_inout
 *                  Resulting finite horizontal segment; written only when the
 *                  wall is usable.
 *
 * @param[out]      isBuilt_out
 *                  True when the wall provides a valid finite segment.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
SemanticsManagerStatus
    buildFiniteWallSegment2d(geometric::Plane      *p_wall_in,
                             const Eigen::Vector3d &groundNormal_world_in,
                             const Eigen::Vector3d &groundAxisU_world_in,
                             const Eigen::Vector3d &groundAxisV_world_in,
                             const double           endpointTrimRatio_in,
                             const double           minimumWallLength_m_in,
                             FiniteWallSegment2d   &segment_inout,
                             bool                  &isBuilt_out)
{
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
    if (p_wall_in == nullptr || wallIsBad)
    {
        isBuilt_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    geometric::Plane::GeometrySnapshot wallGeometry{};
    if (p_wall_in->getGeometrySnapshot(wallGeometry) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometrySnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d wallEquation_world = wallGeometry.equation_world;
    const double    wallNormalNorm     = wallEquation_world.head<3>().norm();

    if (!wallEquation_world.allFinite() || wallNormalNorm < 1e-8)
    {
        isBuilt_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    wallEquation_world /= wallNormalNorm;

    Eigen::Vector3d horizontalWallNormal_world =
        wallEquation_world.head<3>() -
        wallEquation_world.head<3>().dot(groundNormal_world_in) *
            groundNormal_world_in;

    const double horizontalWallNormalNorm = horizontalWallNormal_world.norm();

    if (!std::isfinite(horizontalWallNormalNorm) ||
        horizontalWallNormalNorm < 1e-8)
    {
        isBuilt_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    horizontalWallNormal_world /= horizontalWallNormalNorm;

    const Eigen::Vector3d wallTangent_world =
        groundNormal_world_in.cross(horizontalWallNormal_world).normalized();

    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallSupportCloud =
        wallGeometry.supportCloud;

    if (p_wallSupportCloud == nullptr || p_wallSupportCloud->empty())
    {
        isBuilt_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::vector<double> wallPointCoordinates_m;
    wallPointCoordinates_m.reserve(p_wallSupportCloud->size());

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

        wallPointCoordinates_m.push_back(
            wallPoint_world_m.dot(wallTangent_world));
    }

    if (wallPointCoordinates_m.size() < 2U)
    {
        isBuilt_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::sort(wallPointCoordinates_m.begin(), wallPointCoordinates_m.end());

    const double boundedTrimRatio = std::clamp(endpointTrimRatio_in, 0.0, 0.45);
    const std::size_t trimmedPointCount = static_cast<std::size_t>(
        std::floor(boundedTrimRatio *
                   static_cast<double>(wallPointCoordinates_m.size() - 1U)));
    const std::size_t maximumCoordinateIndex =
        wallPointCoordinates_m.size() - 1U - trimmedPointCount;

    const double minimumWallCoordinate_m =
        wallPointCoordinates_m[trimmedPointCount];
    const double maximumWallCoordinate_m =
        wallPointCoordinates_m[maximumCoordinateIndex];

    Eigen::Vector3d wallGetCentroid{};
    if (p_wall_in->getCentroid(wallGetCentroid) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector3d wallCentroid_world_m = wallGetCentroid.cast<double>();

    if (!wallCentroid_world_m.allFinite())
    {
        isBuilt_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const double centroidCoordinate_m =
        wallCentroid_world_m.dot(wallTangent_world);
    const Eigen::Vector3d segmentStart_world_m =
        wallCentroid_world_m +
        (minimumWallCoordinate_m - centroidCoordinate_m) * wallTangent_world;
    const Eigen::Vector3d segmentEnd_world_m =
        wallCentroid_world_m +
        (maximumWallCoordinate_m - centroidCoordinate_m) * wallTangent_world;

    segment_inout.p_wall        = p_wall_in;
    segment_inout.start_world_m = {
        segmentStart_world_m.dot(groundAxisU_world_in),
        segmentStart_world_m.dot(groundAxisV_world_in)};
    segment_inout.end_world_m = {segmentEnd_world_m.dot(groundAxisU_world_in),
                                 segmentEnd_world_m.dot(groundAxisV_world_in)};
    segment_inout.length_m =
        (segment_inout.end_world_m - segment_inout.start_world_m).norm();
    std::size_t wallGetObservationCount{};
    if (p_wall_in->getObservationCount(wallGetObservationCount) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservationCount returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    segment_inout.supportScore =
        static_cast<double>(std::max<std::size_t>(
            static_cast<std::size_t>(wallGetObservationCount),
            1U)) *
        std::sqrt(std::max(segment_inout.length_m, 0.0));

    isBuilt_out = segment_inout.start_world_m.allFinite() &&
                  segment_inout.end_world_m.allFinite() &&
                  std::isfinite(segment_inout.length_m) &&
                  segment_inout.length_m >= minimumWallLength_m_in;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
