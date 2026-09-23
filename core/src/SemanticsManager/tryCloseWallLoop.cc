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

namespace vs_graphs
{
namespace core
{

WallLoopClosure tryCloseWallLoop(
    std::vector<FiniteWallSegment2d> wallSegments_in,
    const Eigen::Vector2d           &roomCentroid_Ground_m_in,
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters_in)
{
    WallLoopClosure result;

    if (wallSegments_in.empty())
    {
        return result;
    }

    std::sort(
        wallSegments_in.begin(),
        wallSegments_in.end(),
        [&roomCentroid_Ground_m_in](const FiniteWallSegment2d &firstSegment,
                                    const FiniteWallSegment2d &secondSegment)
        {
            const Eigen::Vector2d firstMidpoint =
                0.5 * (firstSegment.start_World_m + firstSegment.end_World_m) -
                roomCentroid_Ground_m_in;
            const Eigen::Vector2d secondMidpoint =
                0.5 *
                    (secondSegment.start_World_m + secondSegment.end_World_m) -
                roomCentroid_Ground_m_in;

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

        if (intersectSupportingLines(currentWall,
                                     nextWall,
                                     corner_World_m,
                                     currentParameter,
                                     nextParameter))
        {
            const double currentCornerGap_m =
                pointToSegmentDistance_m(corner_World_m, currentWall);
            const double nextCornerGap_m =
                pointToSegmentDistance_m(corner_World_m, nextWall);

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

        auto nearestEndpointPair = std::min_element(
            endpointPairs.begin(),
            endpointPairs.end(),
            [](const auto &firstPair, const auto &secondPair)
            {
                return (firstPair.first - firstPair.second).squaredNorm() <
                       (secondPair.first - secondPair.second).squaredNorm();
            });

        if ((nearestEndpointPair->first - nearestEndpointPair->second).norm() <=
            topologyParameters_in.maximumCornerGap_m)
        {
            result.corners_World_m.push_back(
                0.5 *
                (nearestEndpointPair->first + nearestEndpointPair->second));
            continue;
        }

        result.corners_World_m.clear();
        return result;
    }

    if (result.corners_World_m.size() == wallSegments_in.size())
    {
        result.hasOpenBoundary = false;
    }
    else
    {
        result.corners_World_m.clear();
    }

    return result;
}

} // namespace core
} // namespace vs_graphs
