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
 * @file            computeRoomWallFootprint.cc
 *
 * @brief           Implements computeRoomWallFootprint(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "private_functions.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus computeRoomWallFootprint(
    const std::vector<geometric::Plane *> &roomWalls_in,
    const Eigen::Vector3d                 &upDirection_world_in,
    RoomWallFootprint                     &footprint_out,
    bool                                  &hasFootprint_out)
{
    hasFootprint_out    = false;
    const double upNorm = upDirection_world_in.norm();
    if (!upDirection_world_in.allFinite() || upNorm < 1e-8)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    const Eigen::Vector3d upDirection_world = upDirection_world_in / upNorm;

    std::vector<Eigen::Vector3d> horizontalNormals_world{};
    std::vector<Eigen::Vector3d> wallCorners_world_m{};
    std::size_t                  longestWallIndex    = 0U;
    double                       longestWallLength_m = -1.0;

    for (geometric::Plane *p_wall : roomWalls_in)
    {
        bool wallIsBad{};
        if (p_wall != nullptr &&
            p_wall->isBad(wallIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_wall == nullptr || wallIsBad)
        {
            continue;
        }

        /* The same extents the S-Graph archive writes for this wall. */
        geometric::PlaneGeometryMetadataSnapshot wallGeometry{};
        if (p_wall->getGeometryMetadataSnapshot(wallGeometry) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGeometryMetadataSnapshot returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        Eigen::Vector3d wallNormal_world =
            wallGeometry.planeEquation_world.head<3>();
        const double           wallNormalNorm = wallNormal_world.norm();
        const Eigen::Vector3d &wallCentroid_world_m =
            wallGeometry.planeCentroid_world_m;
        /* Unset extents are the max/lowest sentinels, so min > max. */
        const bool hasExtents =
            std::isfinite(wallGeometry.minPlaneU_m) &&
            std::isfinite(wallGeometry.maxPlaneU_m) &&
            std::isfinite(wallGeometry.minPlaneV_m) &&
            std::isfinite(wallGeometry.maxPlaneV_m) &&
            wallGeometry.minPlaneU_m <= wallGeometry.maxPlaneU_m &&
            wallGeometry.minPlaneV_m <= wallGeometry.maxPlaneV_m;
        if (!wallNormal_world.allFinite() || wallNormalNorm < 1e-8 ||
            !wallCentroid_world_m.allFinite() || !hasExtents)
        {
            continue;
        }
        wallNormal_world /= wallNormalNorm;

        /*!
         * The extents are absolute projections onto the axes they were
         * measured on (stored with them), so the plane origin is the
         * centroid without its in-plane part. Rebuilding the axes from the
         * current equation could pick another basis and misplace the box.
         */
        const Eigen::Vector3d &wallAxisU_world =
            wallGeometry.boundsAxes.planeAxisU_world;
        const Eigen::Vector3d &wallAxisV_world =
            wallGeometry.boundsAxes.planeAxisV_world;
        const Eigen::Vector3d planeOrigin_world_m =
            wallCentroid_world_m -
            wallCentroid_world_m.dot(wallAxisU_world) * wallAxisU_world -
            wallCentroid_world_m.dot(wallAxisV_world) * wallAxisV_world;

        /* A plane facing up or down bounds no floor area. */
        Eigen::Vector3d horizontalNormal_world =
            wallNormal_world -
            wallNormal_world.dot(upDirection_world) * upDirection_world;
        if (horizontalNormal_world.norm() < 1e-3)
        {
            continue;
        }
        horizontalNormal_world.normalize();

        const std::array<std::array<double, 2>, 4> cornerCoordinates_m = {
            {{wallGeometry.minPlaneU_m, wallGeometry.minPlaneV_m},
             {wallGeometry.maxPlaneU_m, wallGeometry.minPlaneV_m},
             {wallGeometry.maxPlaneU_m, wallGeometry.maxPlaneV_m},
             {wallGeometry.minPlaneU_m, wallGeometry.maxPlaneV_m}}};
        const Eigen::Vector3d wallTangent_world =
            upDirection_world.cross(horizontalNormal_world);
        double minimumAlongWall_m = std::numeric_limits<double>::infinity();
        double maximumAlongWall_m = -std::numeric_limits<double>::infinity();
        for (const std::array<double, 2> &cornerCoordinate_m :
             cornerCoordinates_m)
        {
            const Eigen::Vector3d wallCorner_world_m =
                planeOrigin_world_m + cornerCoordinate_m[0] * wallAxisU_world +
                cornerCoordinate_m[1] * wallAxisV_world;
            wallCorners_world_m.push_back(wallCorner_world_m);
            const double alongWall_m =
                wallCorner_world_m.dot(wallTangent_world);
            minimumAlongWall_m = std::min(minimumAlongWall_m, alongWall_m);
            maximumAlongWall_m = std::max(maximumAlongWall_m, alongWall_m);
        }

        if (maximumAlongWall_m - minimumAlongWall_m > longestWallLength_m)
        {
            longestWallLength_m = maximumAlongWall_m - minimumAlongWall_m;
            longestWallIndex    = horizontalNormals_world.size();
        }
        horizontalNormals_world.push_back(horizontalNormal_world);
    }

    if (horizontalNormals_world.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /*!
     * Walls span a box only when one of them crosses the longest wall: a
     * single wall, or only parallel walls, leave the box open along the
     * walls' own direction. Crossing means closer to perpendicular than to
     * parallel (more than 45 degrees apart).
     */
    const Eigen::Vector3d firstAxis_world =
        horizontalNormals_world[longestWallIndex];
    const double crossingCosineLimit = std::cos(M_PI / 4.0);
    const bool   doWallsSpanBox      = std::any_of(
        horizontalNormals_world.begin(),
        horizontalNormals_world.end(),
        [&firstAxis_world,
         crossingCosineLimit](const Eigen::Vector3d &horizontalNormal_world)
        {
            return std::abs(horizontalNormal_world.dot(firstAxis_world)) <
                   crossingCosineLimit;
        });
    if (!doWallsSpanBox)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    RoomWallFootprint footprint{};
    footprint.firstAxis_world = firstAxis_world;
    footprint.secondAxis_world =
        upDirection_world.cross(firstAxis_world).normalized();
    footprint.minimumFirst_m  = std::numeric_limits<double>::infinity();
    footprint.maximumFirst_m  = -std::numeric_limits<double>::infinity();
    footprint.minimumSecond_m = std::numeric_limits<double>::infinity();
    footprint.maximumSecond_m = -std::numeric_limits<double>::infinity();
    for (const Eigen::Vector3d &wallCorner_world_m : wallCorners_world_m)
    {
        const double alongFirst_m =
            wallCorner_world_m.dot(footprint.firstAxis_world);
        const double alongSecond_m =
            wallCorner_world_m.dot(footprint.secondAxis_world);
        footprint.minimumFirst_m =
            std::min(footprint.minimumFirst_m, alongFirst_m);
        footprint.maximumFirst_m =
            std::max(footprint.maximumFirst_m, alongFirst_m);
        footprint.minimumSecond_m =
            std::min(footprint.minimumSecond_m, alongSecond_m);
        footprint.maximumSecond_m =
            std::max(footprint.maximumSecond_m, alongSecond_m);
    }

    footprint_out    = footprint;
    hasFootprint_out = true;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
