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
 * @file            excludeOffLoopOutlierWall.cc
 *
 * @brief           Implements excludeOffLoopOutlierWall(), declared in
 *                  room_validation/private_functions/private_functions.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/room_validation/private_functions/private_functions.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <numeric>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus excludeOffLoopOutlierWall(
    semantic::Room        *p_room_in,
    const Eigen::Vector2d &roomCentroidGround_m_in,
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters_in,
    std::vector<FiniteWallSegment2d>                     &wallSegments_inout,
    WallLoopClosure                                      &closure_inout)
{
    std::vector<std::size_t> indicesBySupportAscending(
        wallSegments_inout.size());
    std::iota(indicesBySupportAscending.begin(),
              indicesBySupportAscending.end(),
              0U);
    /* Lambda kept: std::sort needs a comparator that reads the segments;
     * the same comparator keeps the order of equally supported walls, which
     * decides the excluded wall. */
    std::sort(
        indicesBySupportAscending.begin(),
        indicesBySupportAscending.end(),
        [&wallSegments_inout](std::size_t firstIndex, std::size_t secondIndex)
        {
            return wallSegments_inout[firstIndex].supportScore <
                   wallSegments_inout[secondIndex].supportScore;
        });

    for (std::size_t excludeIndex : indicesBySupportAscending)
    {
        std::vector<FiniteWallSegment2d> reducedWallSegments;
        reducedWallSegments.reserve(wallSegments_inout.size() - 1U);
        for (std::size_t segmentIndex = 0U;
             segmentIndex < wallSegments_inout.size();
             ++segmentIndex)
        {
            if (segmentIndex != excludeIndex)
            {
                reducedWallSegments.push_back(wallSegments_inout[segmentIndex]);
            }
        }

        WallLoopClosure reducedClosure{};
        if (tryCloseWallLoop(reducedWallSegments,
                             roomCentroidGround_m_in,
                             topologyParameters_in,
                             reducedClosure) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: tryCloseWallLoop returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (!reducedClosure.hasOpenBoundary)
        {
            int roomId3{};
            if (p_room_in->getId(roomId3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int getId2{};
            if (wallSegments_inout[excludeIndex].p_wall->getId(getId2) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Room#" << roomId3 << ": excluding Wall#"
                      << getId2 << " lets the remaining "
                      << reducedWallSegments.size()
                      << " wall(s) close a valid loop; treating it as an "
                         "off-loop outlier."
                      << std::endl;
            closure_inout      = reducedClosure;
            wallSegments_inout = reducedWallSegments;
            break;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
