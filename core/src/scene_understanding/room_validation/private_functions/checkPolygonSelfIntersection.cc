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
 * @file            checkPolygonSelfIntersection.cc
 *
 * @brief           Implements checkPolygonSelfIntersection(), declared in
 *                  room_validation/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/room_validation/private_functions/private_functions.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus checkPolygonSelfIntersection(
    const std::vector<Eigen::Vector2d> &boundaryCorners_world_m_in,
    bool                               &polygonSelfIntersects_out)
{
    polygonSelfIntersects_out = false;

    for (std::size_t firstEdgeIndex = 0U;
         firstEdgeIndex < boundaryCorners_world_m_in.size() &&
         !polygonSelfIntersects_out;
         ++firstEdgeIndex)
    {
        FiniteWallSegment2d firstBoundaryEdge;
        firstBoundaryEdge.wallStart_world_m =
            boundaryCorners_world_m_in[firstEdgeIndex];
        firstBoundaryEdge.wallEnd_world_m =
            boundaryCorners_world_m_in[(firstEdgeIndex + 1U) %
                                       boundaryCorners_world_m_in.size()];

        for (std::size_t secondEdgeIndex = firstEdgeIndex + 1U;
             secondEdgeIndex < boundaryCorners_world_m_in.size();
             ++secondEdgeIndex)
        {
            const bool edgesAreAdjacent =
                secondEdgeIndex == firstEdgeIndex + 1U ||
                (firstEdgeIndex == 0U &&
                 secondEdgeIndex + 1U == boundaryCorners_world_m_in.size());

            if (edgesAreAdjacent)
            {
                continue;
            }

            FiniteWallSegment2d secondBoundaryEdge;
            secondBoundaryEdge.wallStart_world_m =
                boundaryCorners_world_m_in[secondEdgeIndex];
            secondBoundaryEdge.wallEnd_world_m =
                boundaryCorners_world_m_in[(secondEdgeIndex + 1U) %
                                           boundaryCorners_world_m_in.size()];

            Eigen::Vector2d lineIntersection_world_m;
            double          firstParameter  = 0.0;
            double          secondParameter = 0.0;

            bool hasIntersection2{};
            if (intersectSupportingLines(firstBoundaryEdge,
                                         secondBoundaryEdge,
                                         lineIntersection_world_m,
                                         firstParameter,
                                         secondParameter,
                                         hasIntersection2) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: intersectSupportingLines returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            if (hasIntersection2 && firstParameter > 1e-6 &&
                firstParameter < 1.0 - 1e-6 && secondParameter > 1e-6 &&
                secondParameter < 1.0 - 1e-6)
            {
                polygonSelfIntersects_out = true;
                break;
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
