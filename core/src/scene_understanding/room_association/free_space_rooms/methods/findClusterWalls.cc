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
 * @file            findClusterWalls.cc
 *
 * @brief           Implements SemanticsManager::findClusterWalls(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
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

SemanticsManagerStatus SemanticsManager::findClusterWalls(
    const std::vector<Eigen::Vector3d>    &cluster_in,
    const Eigen::Vector3d                 &clusterCentroid_in,
    const std::vector<geometric::Plane *> &allWalls_in,
    std::vector<geometric::Plane *>       &closestWalls_out)
{
    /* Initialize a list of planes to track the closest walls */
    closestWalls_out.reserve(allWalls_in.size());

    /* For each wall */
    for (vs_graphs::core::geometric::Plane *p_wall : allWalls_in)
    {
        bool isWallSupported = false;
        if (isWallSupportedByCluster(p_wall,
                                     cluster_in,
                                     clusterCentroid_in,
                                     isWallSupported) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: isWallSupportedByCluster returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (isWallSupported)
        {
            closestWalls_out.push_back(p_wall);
        }
    }

    /* Organise closest walls in order of ids */
    std::sort(closestWalls_out.begin(),
              closestWalls_out.end(),
              semantic::isEntityIdLess<geometric::Plane>);

    /* Remove duplicate walls using IDs rather than pointers */
    closestWalls_out.erase(
        std::unique(closestWalls_out.begin(),
                    closestWalls_out.end(),
                    semantic::isEntityIdEqual<geometric::Plane>),
        closestWalls_out.end());

    /* Admit nearer walls first (see admitWallToRoom()'s see-through
     * gate); the id order above only serves the duplicate removal. */
    if (sortWallsByCameraDistance(closestWalls_out) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: sortWallsByCameraDistance returned a failure "
                     "status although it cannot fail; continuing as "
                     "before.",
                     __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
