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
 * @file            associateWallsToRoomsFromFreeSpaceClusters.cc
 *
 * @brief           Implements
 * SemanticsManager::associateWallsToRoomsFromFreeSpaceClusters(), declared in
 * SemanticsManagerClass.h.
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

SemanticsManagerStatus
    SemanticsManager::associateWallsToRoomsFromFreeSpaceClusters(void)
{
    PROFILE_FUNCTION();
    /* Extract latest skeleton cluster */
    std::vector<std::vector<Eigen::Vector3d>> latestSkeletonCluster{};
    if (getLatestSkeletonCluster(latestSkeletonCluster) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLatestSkeletonCluster returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<std::vector<Eigen::Vector3d>> clusters{};
    if (partitionFreeSpaceAtPassages(latestSkeletonCluster, clusters) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: partitionFreeSpaceAtPassages returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    /* If cluster is empty then return */
    if (clusters.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract all planes from the map */
    std::vector<vs_graphs::core::geometric::Plane *> allPlanes{};
    if (p_atlas->getAllPlanes(allPlanes) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Create a list of all the walls there are in the SGraph */
    std::vector<vs_graphs::core::geometric::Plane *> allWalls;
    if (collectAdmissibleWalls(allPlanes, allWalls) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: collectAdmissibleWalls returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* If there are no walls, then return */
    if (allWalls.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Track room IDs matched in this cycle to avoid double-matching */
    std::unordered_set<int> matchedRoomIds;

    /* Iterate through all the clusters */
    for (std::size_t clusterId = 0; clusterId < clusters.size(); clusterId++)
    {
        /* Extract cluster */
        const std::vector<Eigen::Vector3d> &cluster = clusters[clusterId];

        /* If cluster is empty skip */
        if (cluster.empty())
        {
            continue;
        }

        /* Extract the cluster centroid */
        Eigen::Vector3d clusterCentroid{};
        if (utils::utils::Utils::computeCentroidFromPoints(cluster,
                                                           clusterCentroid) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: computeCentroidFromPoints returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Find the walls the cluster supports, nearest first */
        std::vector<vs_graphs::core::geometric::Plane *> closestWalls{};
        if (findClusterWalls(cluster,
                             clusterCentroid,
                             allWalls,
                             closestWalls) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findClusterWalls returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* If there are no closest walls then skip to next cluster */
        if (closestWalls.empty())
        {
            continue;
        }

        /*
         * Match external far-side evidence before admitting any new walls.
         * Taking the wall snapshot here prevents admission by this cluster
         * from manufacturing its own promotion evidence.
         */
        FarSideProspectiveMatch prospectiveMatch{};
        if (findFarSideProspectiveRoom(allPlanes,
                                       clusterCentroid,
                                       closestWalls,
                                       matchedRoomIds,
                                       prospectiveMatch) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: findFarSideProspectiveRoom returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Resolve the room this cluster describes; defer when none. */
        vs_graphs::core::semantic::Room *p_room = nullptr;
        if (resolveClusterRoom(clusterId,
                               cluster,
                               clusterCentroid,
                               closestWalls,
                               prospectiveMatch.p_prospectiveRoom,
                               matchedRoomIds,
                               p_room) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: resolveClusterRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr)
        {
            if (reportDeferredCluster(clusterId) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: reportDeferredCluster returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            continue;
        }

        /* The room centre is owned by its walls, not by free space: rooms
         * keep their creation centroid until walls arrive, then track the
         * damped wall-centroid mean in the consolidation below. Overwriting
         * from the cluster centroid every cycle drags the centre toward
         * whichever free space was observed last (live-observed: across a
         * passage onto its far side) and couples maintenance to Voxblox
         * liveness. Free-space evidence places a room once, at creation. */

        bool roomIsPassageBoundProspective = false;
        if (isRoomPassageBoundProspective(p_room,
                                          roomIsPassageBoundProspective) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: isRoomPassageBoundProspective returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* A prospective must first pass the external cluster validation below;
         * generic consolidation must not classify or replace it early. */
        if (!roomIsPassageBoundProspective)
        {
            if (consolidateRoomsInFreeSpaceCluster(p_room, cluster, allWalls) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: consolidateRoomsInFreeSpaceCluster returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        /* Admit the cluster's walls to the room */
        if (admitClusterWallsToRoom(p_room, closestWalls, clusterCentroid) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: admitClusterWallsToRoom returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Consolidate the room's walls and update its centre */
        std::vector<geometric::Plane *> roomWalls{};
        if (refreshClusterRoomGeometry(p_room, roomWalls) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: refreshClusterRoomGeometry returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Confirm the room from its free-space cluster */
        if (confirmClusterRoom(clusterId,
                               cluster,
                               p_room,
                               roomWalls,
                               closestWalls,
                               prospectiveMatch,
                               roomIsPassageBoundProspective) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: confirmClusterRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
