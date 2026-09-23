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

namespace vs_graphs
{
namespace core
{

bool segmentCrossesForeignWall(
    const Eigen::Vector3d &segmentStart_World_m_in,
    const Eigen::Vector3d &segmentEnd_World_m_in,
    const std::vector<vs_graphs::core::semantic::Room *> &excludedRooms_in,
    const std::vector<vs_graphs::core::semantic::Room *> &allRooms_in,
    const Eigen::Vector3d                                &groundAxisU_World_in,
    const Eigen::Vector3d                                &groundAxisV_World_in,
    const Eigen::Vector3d                                &groundNormal_World_in,
    const double                                          endpointTrimRatio_in,
    const double minimumWallLength_m_in)
{
    if (!segmentStart_World_m_in.allFinite() ||
        !segmentEnd_World_m_in.allFinite())
    {
        return false;
    }

    FiniteWallSegment2d testSegment;
    testSegment.start_World_m = {
        segmentStart_World_m_in.dot(groundAxisU_World_in),
        segmentStart_World_m_in.dot(groundAxisV_World_in)};
    testSegment.end_World_m = {segmentEnd_World_m_in.dot(groundAxisU_World_in),
                               segmentEnd_World_m_in.dot(groundAxisV_World_in)};

    for (vs_graphs::core::semantic::Room *p_room : allRooms_in)
    {
        if (p_room == nullptr || p_room->isBad() ||
            std::find(excludedRooms_in.begin(),
                      excludedRooms_in.end(),
                      p_room) != excludedRooms_in.end())
        {
            continue;
        }

        for (geometric::Plane *p_wall : p_room->getWalls())
        {
            FiniteWallSegment2d wallSegment;

            if (!buildFiniteWallSegment2d(p_wall,
                                          groundNormal_World_in,
                                          groundAxisU_World_in,
                                          groundAxisV_World_in,
                                          endpointTrimRatio_in,
                                          minimumWallLength_m_in,
                                          wallSegment))
            {
                continue;
            }

            Eigen::Vector2d intersection_World_m;
            double          testParameter = 0.0;
            double          wallParameter = 0.0;

            if (intersectSupportingLines(testSegment,
                                         wallSegment,
                                         intersection_World_m,
                                         testParameter,
                                         wallParameter) &&
                testParameter > 0.0 && testParameter < 1.0 &&
                wallParameter >= 0.0 && wallParameter <= 1.0)
            {
                return true;
            }
        }
    }

    return false;
}

} // namespace core
} // namespace vs_graphs
