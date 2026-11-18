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
 * @file            hasWallLiveClusterSupport.cc
 *
 * @brief           Implements SemanticsManager::hasWallLiveClusterSupport(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <rclcpp/logging.hpp>
#include <unordered_set>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::hasWallLiveClusterSupport(
    geometric::Plane                                *p_wall_in,
    const std::vector<std::vector<Eigen::Vector3d>> &skeletonClusters_in,
    bool &hasCompatibleLiveCluster_out)
{
    geometric::Plane::GeometrySnapshot wallGeometry{};
    if (p_wall_in->getGeometrySnapshot(wallGeometry) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometrySnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector4d wallEquation_world = wallGeometry.planeEquation_world;
    const double          wallNormalNorm = wallEquation_world.head<3>().norm();
    hasCompatibleLiveCluster_out         = false;
    if (wallGeometry.planeCentroid_world_m.allFinite() &&
        wallEquation_world.allFinite() && wallNormalNorm > 1e-8)
    {
        const Eigen::Vector3d wallNormal_world =
            wallEquation_world.head<3>() / wallNormalNorm;
        const double wallOffset_m = wallEquation_world(3) / wallNormalNorm;
        const double maximumCentroidDistance_m =
            2.0 *
            static_cast<double>(
                p_sysParams->roomSeg.clusterCentroidWallCentroidDistanceThresh);
        const double maximumPointDistance_m = static_cast<double>(
            p_sysParams->roomSeg.clusterPointWallDistanceThresh);

        for (const std::vector<Eigen::Vector3d> &cluster : skeletonClusters_in)
        {
            if (cluster.empty())
            {
                continue;
            }
            Eigen::Vector3d clusterCentroid_world_m{};
            if (utils::utils::Utils::computeCentroidFromPoints(
                    cluster,
                    clusterCentroid_world_m) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                /*!
                 * computeCentroidFromPoints cannot fail; continue as
                 * before.
                 */
            }
            if (!clusterCentroid_world_m.allFinite() ||
                (clusterCentroid_world_m - wallGeometry.planeCentroid_world_m)
                        .norm() > maximumCentroidDistance_m)
            {
                continue;
            }
            for (const Eigen::Vector3d &point : cluster)
            {
                if (point.allFinite() &&
                    std::abs(wallNormal_world.dot(point) + wallOffset_m) <=
                        maximumPointDistance_m)
                {
                    hasCompatibleLiveCluster_out = true;
                    break;
                }
            }
            if (hasCompatibleLiveCluster_out)
            {
                break;
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
