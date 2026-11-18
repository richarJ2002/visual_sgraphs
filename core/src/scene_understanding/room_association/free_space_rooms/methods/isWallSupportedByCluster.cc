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
 * @file            isWallSupportedByCluster.cc
 *
 * @brief           Implements SemanticsManager::isWallSupportedByCluster(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/objects/SemanticsManagerClass.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::isWallSupportedByCluster(
    geometric::Plane                   *p_wall_in,
    const std::vector<Eigen::Vector3d> &cluster_in,
    const Eigen::Vector3d              &clusterCentroid_in,
    bool                               &isSupported_out) const
{
    isSupported_out = false;

    /* Skip wall if it is bad */
    bool wallIsBad{};
    if (!(p_wall_in == nullptr) &&
        p_wall_in->isBad(wallIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_wall_in == nullptr || wallIsBad)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract the point cloud for the wall */
    geometric::Plane::GeometrySnapshot wallGeometry{};
    if (p_wall_in->getGeometrySnapshot(wallGeometry) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometrySnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallCloud =
        wallGeometry.supportCloud;

    /* Skip wall if the point cloud is invalid */
    if (p_wallCloud == nullptr || p_wallCloud->empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract the centroid of the wall */
    const Eigen::Vector3d wallCentroid = wallGeometry.planeCentroid_world_m;

    /* Find the distance from the wall centroid and cluster centroid */
    const double centroidDistance = (wallCentroid - clusterCentroid_in).norm();

    /*!
     * Use centroid distance only as a coarse rejection condition.
     *
     * @note            A long wall may have a centroid far from the
     *                  room centre while still forming a valid boundary
     *                  of the room.
     */
    const double coarseCentroidDistanceThreshold =
        2.0 *
        static_cast<double>(
            p_sysParams->roomSeg.clusterCentroidWallCentroidDistanceThresh);

    if (centroidDistance >= coarseCentroidDistanceThreshold)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract plane equation for the wall */
    g2o::Plane3D wallGetGlobalEquation{};
    if (p_wall_in->getGlobalEquation(wallGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d wallEquation = wallGetGlobalEquation.coeffs();

    /* Extract the norm of the normal of the wall */
    const double normalMagnitude = wallEquation.head<3>().norm();

    /* Skip wall if norm is invalid */
    if (!std::isfinite(normalMagnitude) || normalMagnitude < 1e-8)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Find the normal vector of the wall */
    const Eigen::Vector3d wallNormalVector = wallEquation.head<3>();

    /* Find distance to normal */
    const double wallDistance = wallEquation(3) / normalMagnitude;

    /* Find the normalized vector */
    const Eigen::Vector3d wallNormUnitVector =
        wallNormalVector / normalMagnitude;

    /* Find an axis U tangent to the wall plane. */
    const Eigen::Vector3d axisU =
        wallNormUnitVector.unitOrthogonal().normalized();

    /* Find the orthogonal axis completing the wall-plane basis. */
    const Eigen::Vector3d axisV = wallNormUnitVector.cross(axisU).normalized();

    /* Measure finite wall bounds in the same basis used below. */
    std::size_t validWallPoints = 0;
    double      minimumWallU_m  = std::numeric_limits<double>::max();
    double      maximumWallU_m  = std::numeric_limits<double>::lowest();
    double      minimumWallV_m  = std::numeric_limits<double>::max();
    double      maximumWallV_m  = std::numeric_limits<double>::lowest();

    /* Iterate through each point in the wall point cloud */
    for (const pcl::PointXYZRGBA &point : p_wallCloud->points)
    {
        /* If point is invalid, skip */
        if (!pcl::isFinite(point))
        {
            continue;
        }

        const Eigen::Vector3d wallPoint_world_m(static_cast<double>(point.x),
                                                static_cast<double>(point.y),
                                                static_cast<double>(point.z));

        const Eigen::Vector3d wallPointRelToCentroid_world_m =
            wallPoint_world_m - wallCentroid;

        const double wallPointU_m = wallPointRelToCentroid_world_m.dot(axisU);
        const double wallPointV_m = wallPointRelToCentroid_world_m.dot(axisV);

        minimumWallU_m = std::min(minimumWallU_m, wallPointU_m);
        maximumWallU_m = std::max(maximumWallU_m, wallPointU_m);
        minimumWallV_m = std::min(minimumWallV_m, wallPointV_m);
        maximumWallV_m = std::max(maximumWallV_m, wallPointV_m);

        validWallPoints++;
    }

    /* A finite wall cannot be measured without a valid cloud point. */
    if (validWallPoints == 0)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Confirm wall bounds are valid, otherwise skip */
    if (!std::isfinite(minimumWallU_m) || !std::isfinite(maximumWallU_m) ||
        !std::isfinite(minimumWallV_m) || !std::isfinite(maximumWallV_m))
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const double wallExtentU_m = maximumWallU_m - minimumWallU_m;
    const double wallExtentV_m = maximumWallV_m - minimumWallV_m;

    /* Skip small fragments without erasing their semantic evidence. */
    if (wallExtentU_m < p_sysParams->roomSeg.minimumFiniteWallExtent_m ||
        wallExtentV_m < p_sysParams->roomSeg.minimumFiniteWallExtent_m)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /*!
     * Number of close points whose projections lie inside the finite
     * wall patch.
     */
    std::size_t supportedPointCount = 0;
    std::size_t nearbyPointCount    = 0;

    /* Init variable to track minimum distance from wall and cluster */
    double minimumPlaneDistance = std::numeric_limits<double>::max();

    /* Iterate through each point in cluster to find point in wall */
    for (const Eigen::Vector3d &clusterPoint : cluster_in)
    {
        /* Find the signed distance along wall normal*/
        const double signedPlaneDistance =
            wallNormUnitVector.dot(clusterPoint) + wallDistance;

        /* Find the absolute value of the plane distance */
        const double planeDistance = std::abs(signedPlaneDistance);

        /* Keep the smallest observed plane distance. */
        minimumPlaneDistance = std::min(minimumPlaneDistance, planeDistance);

        /* If the plane distance is larger than threshold, skip point */
        if (planeDistance >=
            p_sysParams->roomSeg.clusterPointWallDistanceThresh)
        {
            continue;
        }

        nearbyPointCount++;

        /* Find the projected point on the plane */
        const Eigen::Vector3d projectedPoint =
            clusterPoint - signedPlaneDistance * wallNormUnitVector;

        /* Find relative distance between point and wall centroid */
        const Eigen::Vector3d projectedRelative = projectedPoint - wallCentroid;

        /* Find distance in U axis on wall */
        const double projectedU = projectedRelative.dot(axisU);

        /* Find distance in V axis on wall */
        const double projectedV = projectedRelative.dot(axisV);

        /* Confirm if the wall encapsulates the projected point */
        const bool insideFiniteWall =
            projectedU >= minimumWallU_m -
                              p_sysParams->roomSeg.finiteWallBoundsMargin_m &&
            projectedU <= maximumWallU_m +
                              p_sysParams->roomSeg.finiteWallBoundsMargin_m &&
            projectedV >= minimumWallV_m -
                              p_sysParams->roomSeg.finiteWallBoundsMargin_m &&
            projectedV <=
                maximumWallV_m + p_sysParams->roomSeg.finiteWallBoundsMargin_m;

        /*!
         * Count nearby points projected inside the finite wall patch.
         */
        if (insideFiniteWall)
        {
            supportedPointCount++;
        }
        else
        {
            continue;
        }
    }

    /* If the plane distance from cluster is far from threshold, skip */
    if (minimumPlaneDistance >
        p_sysParams->roomSeg.clusterPointWallDistanceThresh)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Calculate the fraction of nearby points inside finite bounds. */
    const double finiteSupportRatio =
        nearbyPointCount > 0 ? static_cast<double>(supportedPointCount) /
                                   static_cast<double>(nearbyPointCount)
                             : 0.0;

    /*!
     * Accept the wall only when there is sufficient absolute support
     * and the majority of nearby points project inside the finite wall
     * patch.
     */

    if (supportedPointCount >=
            p_sysParams->roomSeg.minimumWallSupportPointCount &&
        finiteSupportRatio >= p_sysParams->roomSeg.minimumWallSupportRatio)
    {
        isSupported_out = true;
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
