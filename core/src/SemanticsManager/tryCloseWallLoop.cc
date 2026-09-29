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
#include <array>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief Attempts to close the given wall segments (sorted here by angle
 *        from the room centroid) into one ordered loop, exactly as
 *        validateRoomBoundaries() always did for a room's full wall set.
 *        Factored out so the caller can retry on a reduced subset when the
 *        full set doesn't close (see validateRoomBoundaries()'s single-
 *        outlier-exclusion retry).
 */
SemanticsManagerStatus tryCloseWallLoop(
    std::vector<FiniteWallSegment2d> wallSegments_in,
    const Eigen::Vector2d           &roomCentroidGround_m_in,
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters_in,
    WallLoopClosure                                      &closure_out)
{
    WallLoopClosure result;

    if (wallSegments_in.empty())
    {
        closure_out = result;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::sort(
        wallSegments_in.begin(),
        wallSegments_in.end(),
        [&roomCentroidGround_m_in](const FiniteWallSegment2d &firstSegment,
                                   const FiniteWallSegment2d &secondSegment)
        {
            const Eigen::Vector2d firstMidpoint =
                0.5 * (firstSegment.start_World_m + firstSegment.end_World_m) -
                roomCentroidGround_m_in;
            const Eigen::Vector2d secondMidpoint =
                0.5 *
                    (secondSegment.start_World_m + secondSegment.end_World_m) -
                roomCentroidGround_m_in;

            return std::atan2(firstMidpoint.y(), firstMidpoint.x()) <
                   std::atan2(secondMidpoint.y(), secondMidpoint.x());
        });

    result.corners_World_m.reserve(wallSegments_in.size());

    for (std::size_t wallIndex = 0U; wallIndex < wallSegments_in.size();
         ++wallIndex)
    {
        const FiniteWallSegment2d &currentWall = wallSegments_in[wallIndex];
        const FiniteWallSegment2d &nextWall =
            wallSegments_in[(wallIndex + 1U) % wallSegments_in.size()];
        Eigen::Vector2d corner_World_m;
        double          currentParameter = 0.0;
        double          nextParameter    = 0.0;

        bool hasIntersection{};
        if (intersectSupportingLines(currentWall,
                                     nextWall,
                                     corner_World_m,
                                     currentParameter,
                                     nextParameter,
                                     hasIntersection) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: intersectSupportingLines returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (hasIntersection)
        {
            double currentCornerGap_m{};
            if (pointToSegmentDistance_m(corner_World_m,
                                         currentWall,
                                         currentCornerGap_m) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: pointToSegmentDistance_m returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            double nextCornerGap_m{};
            if (pointToSegmentDistance_m(corner_World_m,
                                         nextWall,
                                         nextCornerGap_m) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: pointToSegmentDistance_m returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if (currentCornerGap_m <=
                    topologyParameters_in.maximumCornerGap_m &&
                nextCornerGap_m <= topologyParameters_in.maximumCornerGap_m)
            {
                result.corners_World_m.push_back(corner_World_m);
                continue;
            }
        }

        const std::array<std::pair<Eigen::Vector2d, Eigen::Vector2d>, 4>
            endpointPairs = {
                {{currentWall.start_World_m, nextWall.start_World_m},
                 {currentWall.start_World_m, nextWall.end_World_m},
                 {currentWall.end_World_m, nextWall.start_World_m},
                 {currentWall.end_World_m, nextWall.end_World_m}}};

        auto p_nearestEndpointPair = std::min_element(
            endpointPairs.begin(),
            endpointPairs.end(),
            [](const auto &firstPair, const auto &secondPair)
            {
                return (firstPair.first - firstPair.second).squaredNorm() <
                       (secondPair.first - secondPair.second).squaredNorm();
            });

        if ((p_nearestEndpointPair->first - p_nearestEndpointPair->second)
                .norm() <= topologyParameters_in.maximumCornerGap_m)
        {
            result.corners_World_m.push_back(
                0.5 *
                (p_nearestEndpointPair->first + p_nearestEndpointPair->second));
            continue;
        }

        result.corners_World_m.clear();
        closure_out = result;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    if (result.corners_World_m.size() == wallSegments_in.size())
    {
        result.hasOpenBoundary = false;
    }
    else
    {
        result.corners_World_m.clear();
    }

    closure_out = result;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
