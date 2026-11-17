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
 * @file            buildWallSlabQueries.cc
 *
 * @brief           Implements SemanticsManager::buildWallSlabQueries(),
 *                  declared in SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/objects/SemanticsManagerClass.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

#include <rclcpp/logging.hpp>

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::buildWallSlabQueries(
    const std::vector<geometric::Plane *> &confirmedWalls_in)
{
    const types::SystemParams::SemSeg::PassageDetection &passageParameters =
        p_sysParams->semSeg.passageDetection;
    const double cellSize_m =
        static_cast<double>(passageParameters.slabCellSize_m);
    const double wallBoundsMargin_m =
        static_cast<double>(passageParameters.wallBoundsMargin_m);

    /* Rows cover the door band plus head room so the lintel is visible. */
    constexpr double lintelHeadroom_m = 0.3;
    const double     slabHeight_m =
        static_cast<double>(passageParameters.doorBandMaxHeight_m) +
        lintelHeadroom_m;

    std::vector<WallSlabQuery> builtQueries{};

    /* Floor plane oriented so the camera is on its positive side. */
    geometric::Plane *p_groundPlane = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool groundPlaneIsBad = true;
    if ((p_groundPlane != nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d groundEquation_world = Eigen::Vector4d::Zero();
    if (p_groundPlane != nullptr && !groundPlaneIsBad)
    {
        g2o::Plane3D groundGlobalEquation{};
        if (p_groundPlane->getGlobalEquation(groundGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        groundEquation_world = groundGlobalEquation.coeffs();
    }
    const double groundNormalNorm = groundEquation_world.head<3>().norm();
    if (!hasCameraCenter || !groundEquation_world.allFinite() ||
        groundNormalNorm < 1e-8)
    {
        std::lock_guard<std::mutex> wallSlabLock(wallSlabMutex);
        wallSlabQueries.clear();
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    groundEquation_world /= groundNormalNorm;
    if (groundEquation_world.head<3>().dot(currentCameraCenter_world_m) +
            groundEquation_world(3) <
        0.0)
    {
        groundEquation_world = -groundEquation_world;
    }
    const Eigen::Vector3d upDirection_world = groundEquation_world.head<3>();

    for (geometric::Plane *p_wall : confirmedWalls_in)
    {
        if (p_wall == nullptr)
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
        Eigen::Vector4d wallEquation_world = wallGeometry.planeEquation_world;
        const double    wallNormalNorm = wallEquation_world.head<3>().norm();
        if (wallGeometry.supportCloud == nullptr ||
            wallGeometry.supportCloud->empty() ||
            !wallEquation_world.allFinite() || wallNormalNorm < 1e-8)
        {
            continue;
        }
        wallEquation_world /= wallNormalNorm;
        const Eigen::Vector3d wallNormal_world = wallEquation_world.head<3>();

        /* a = normalize(g x n) runs along the wall; b = n x a points up. */
        const Eigen::Vector3d alongCross_world =
            upDirection_world.cross(wallNormal_world);
        if (alongCross_world.norm() < 1e-3)
        {
            continue;
        }
        const Eigen::Vector3d alongAxis_world = alongCross_world.normalized();
        const Eigen::Vector3d upAxis_world =
            wallNormal_world.cross(alongAxis_world);
        const double upAxisRise = upAxis_world.dot(upDirection_world);
        if (upAxisRise < 1e-3)
        {
            continue;
        }

        /* Wall centroid projected onto its plane anchors the slab. */
        const Eigen::Vector3d wallCentroid_world_m =
            wallGeometry.planeCentroid_world_m -
            (wallNormal_world.dot(wallGeometry.planeCentroid_world_m) +
             wallEquation_world(3)) *
                wallNormal_world;
        if (!wallCentroid_world_m.allFinite())
        {
            continue;
        }

        double minimumAlong_m = std::numeric_limits<double>::max();
        double maximumAlong_m = std::numeric_limits<double>::lowest();
        for (const pcl::PointXYZRGBA &supportPoint :
             wallGeometry.supportCloud->points)
        {
            if (!pcl::isFinite(supportPoint))
            {
                continue;
            }
            const double along_m = (Eigen::Vector3d(supportPoint.x,
                                                    supportPoint.y,
                                                    supportPoint.z) -
                                    wallCentroid_world_m)
                                       .dot(alongAxis_world);
            minimumAlong_m = std::min(minimumAlong_m, along_m);
            maximumAlong_m = std::max(maximumAlong_m, along_m);
        }
        if (minimumAlong_m > maximumAlong_m)
        {
            continue;
        }
        minimumAlong_m -= wallBoundsMargin_m;
        maximumAlong_m += wallBoundsMargin_m;

        /* Slide from the centroid along b down to floor height (0 m). */
        const double centroidHeight_m =
            upDirection_world.dot(wallCentroid_world_m) +
            groundEquation_world(3);

        int wallId{};
        if (p_wall->getId(wallId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        WallSlabQuery wallQuery{};
        wallQuery.wallId             = wallId;
        wallQuery.wallEquation_world = wallEquation_world;
        wallQuery.origin_world_m =
            wallCentroid_world_m + minimumAlong_m * alongAxis_world -
            (centroidHeight_m / upAxisRise) * upAxis_world;
        wallQuery.alongAxis_world = alongAxis_world;
        wallQuery.upAxis_world    = upAxis_world;
        wallQuery.upAxisRise      = upAxisRise;
        wallQuery.normal_world    = wallNormal_world;
        wallQuery.columnCount     = static_cast<std::uint32_t>(
            std::ceil((maximumAlong_m - minimumAlong_m) / cellSize_m));
        wallQuery.rowCount = static_cast<std::uint32_t>(
            std::ceil(slabHeight_m / (cellSize_m * upAxisRise)));
        wallQuery.cellSize_m = cellSize_m;
        wallQuery.halfThickness_m =
            static_cast<double>(passageParameters.slabHalfThickness_m);
        builtQueries.push_back(wallQuery);
    }

    std::lock_guard<std::mutex> wallSlabLock(wallSlabMutex);
    wallSlabQueries = std::move(builtQueries);
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
