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
 * @file            segmentCrossesForeignWall.cc
 *
 * @brief           Implements segmentCrossesForeignWall(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "SemanticsManager.h"

#include "private_functions.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus segmentCrossesForeignWall(
    const Eigen::Vector3d &segmentStart_world_m_in,
    const Eigen::Vector3d &segmentEnd_world_m_in,
    const std::vector<vs_graphs::core::semantic::Room *> &excludedRooms_in,
    const std::vector<vs_graphs::core::semantic::Room *> &allRooms_in,
    const Eigen::Vector3d                                &groundAxisU_world_in,
    const Eigen::Vector3d                                &groundAxisV_world_in,
    const Eigen::Vector3d                                &groundNormal_world_in,
    const double                                          endpointTrimRatio_in,
    const double minimumWallLength_m_in,
    bool        &crossesForeignWall_out)
{
    if (!segmentStart_world_m_in.allFinite() ||
        !segmentEnd_world_m_in.allFinite())
    {
        crossesForeignWall_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    FiniteWallSegment2d testSegment;
    testSegment.start_world_m = {
        segmentStart_world_m_in.dot(groundAxisU_world_in),
        segmentStart_world_m_in.dot(groundAxisV_world_in)};
    testSegment.end_world_m = {segmentEnd_world_m_in.dot(groundAxisU_world_in),
                               segmentEnd_world_m_in.dot(groundAxisV_world_in)};

    for (vs_graphs::core::semantic::Room *p_room : allRooms_in)
    {
        bool roomIsBad{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || roomIsBad ||
            std::find(excludedRooms_in.begin(),
                      excludedRooms_in.end(),
                      p_room) != excludedRooms_in.end())
        {
            continue;
        }

        std::vector<geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_wall : roomWalls)
        {
            FiniteWallSegment2d wallSegment;

            bool isBuilt{};
            if (buildFiniteWallSegment2d(p_wall,
                                         groundNormal_world_in,
                                         groundAxisU_world_in,
                                         groundAxisV_world_in,
                                         endpointTrimRatio_in,
                                         minimumWallLength_m_in,
                                         wallSegment,
                                         isBuilt) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: buildFiniteWallSegment2d returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (!isBuilt)
            {
                continue;
            }

            Eigen::Vector2d intersection_world_m;
            double          testParameter = 0.0;
            double          wallParameter = 0.0;

            bool hasIntersection{};
            if (intersectSupportingLines(testSegment,
                                         wallSegment,
                                         intersection_world_m,
                                         testParameter,
                                         wallParameter,
                                         hasIntersection) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: intersectSupportingLines returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (hasIntersection && testParameter > 0.0 && testParameter < 1.0 &&
                wallParameter >= 0.0 && wallParameter <= 1.0)
            {
                crossesForeignWall_out = true;
                return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
            }
        }
    }

    crossesForeignWall_out = false;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
