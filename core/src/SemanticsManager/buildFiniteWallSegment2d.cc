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

namespace vs_graphs
{
namespace core
{

bool buildFiniteWallSegment2d(geometric::Plane      *p_wall_in,
                              const Eigen::Vector3d &groundNormal_World_in,
                              const Eigen::Vector3d &groundAxisU_World_in,
                              const Eigen::Vector3d &groundAxisV_World_in,
                              const double           endpointTrimRatio_in,
                              const double           minimumWallLength_m_in,
                              FiniteWallSegment2d   &segment_out)
{
    if (p_wall_in == nullptr || p_wall_in->isBad())
    {
        return false;
    }

    const geometric::Plane::GeometrySnapshot wallGeometry =
        p_wall_in->getGeometrySnapshot();
    Eigen::Vector4d wallEquation_World = wallGeometry.equation_World;
    const double    wallNormalNorm     = wallEquation_World.head<3>().norm();

    if (!wallEquation_World.allFinite() || wallNormalNorm < 1e-8)
    {
        return false;
    }

    wallEquation_World /= wallNormalNorm;

    Eigen::Vector3d horizontalWallNormal_World =
        wallEquation_World.head<3>() -
        wallEquation_World.head<3>().dot(groundNormal_World_in) *
            groundNormal_World_in;

    const double horizontalWallNormalNorm = horizontalWallNormal_World.norm();

    if (!std::isfinite(horizontalWallNormalNorm) ||
        horizontalWallNormalNorm < 1e-8)
    {
        return false;
    }

    horizontalWallNormal_World /= horizontalWallNormalNorm;

    const Eigen::Vector3d wallTangent_World =
        groundNormal_World_in.cross(horizontalWallNormal_World).normalized();

    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallCloud =
        wallGeometry.supportCloud;

    if (p_wallCloud == nullptr || p_wallCloud->empty())
    {
        return false;
    }

    std::vector<double> wallPointCoordinates_m;
    wallPointCoordinates_m.reserve(p_wallCloud->size());

    for (const pcl::PointXYZRGBA &wallPoint : p_wallCloud->points)
    {
        if (!pcl::isFinite(wallPoint))
        {
            continue;
        }

        const Eigen::Vector3d wallPoint_World_m(
            static_cast<double>(wallPoint.x),
            static_cast<double>(wallPoint.y),
            static_cast<double>(wallPoint.z));

        wallPointCoordinates_m.push_back(
            wallPoint_World_m.dot(wallTangent_World));
    }

    if (wallPointCoordinates_m.size() < 2U)
    {
        return false;
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

    const Eigen::Vector3d wallCentroid_World_m =
        p_wall_in->getCentroid().cast<double>();

    if (!wallCentroid_World_m.allFinite())
    {
        return false;
    }

    const double centroidCoordinate_m =
        wallCentroid_World_m.dot(wallTangent_World);
    const Eigen::Vector3d segmentStart_World_m =
        wallCentroid_World_m +
        (minimumWallCoordinate_m - centroidCoordinate_m) * wallTangent_World;
    const Eigen::Vector3d segmentEnd_World_m =
        wallCentroid_World_m +
        (maximumWallCoordinate_m - centroidCoordinate_m) * wallTangent_World;

    segment_out.p_wall        = p_wall_in;
    segment_out.start_World_m = {
        segmentStart_World_m.dot(groundAxisU_World_in),
        segmentStart_World_m.dot(groundAxisV_World_in)};
    segment_out.end_World_m = {segmentEnd_World_m.dot(groundAxisU_World_in),
                               segmentEnd_World_m.dot(groundAxisV_World_in)};
    segment_out.length_m =
        (segment_out.end_World_m - segment_out.start_World_m).norm();
    segment_out.supportScore =
        static_cast<double>(std::max<std::size_t>(
            static_cast<std::size_t>(p_wall_in->getObservationCount()),
            1U)) *
        std::sqrt(std::max(segment_out.length_m, 0.0));

    return segment_out.start_World_m.allFinite() &&
           segment_out.end_World_m.allFinite() &&
           std::isfinite(segment_out.length_m) &&
           segment_out.length_m >= minimumWallLength_m_in;
}

} // namespace core
} // namespace vs_graphs
