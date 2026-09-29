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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief       Tests whether a straight segment between two points is
 *              blocked by a wall belonging to a room other than the ones
 *              the segment is meant to connect.
 *
 *              Threading one passage's own bounded aperture is necessary
 *              but not sufficient proof that two points are the direct two
 *              sides of THAT passage: in a corridor with several rooms and
 *              doors in a row, a straight line can thread one passage's
 *              opening while still passing directly through an
 *              intervening room's own wall. When it does, something else
 *              -- a wall, and by implication a room -- provably sits
 *              between the two points, so they are not each other's
 *              direct neighbour through this passage.
 *
 * @param[in]   segmentStart_World_m_in
 *              One endpoint of the candidate segment.
 * @param[in]   segmentEnd_World_m_in
 *              The other endpoint of the candidate segment.
 * @param[in]   excludedRooms_in
 *              Rooms whose own walls are not "foreign" -- typically the
 *              rooms/placeholders the segment itself is testing.
 * @param[in]   allRooms_in
 *              Every currently known room to search for a blocking wall.
 * @param[in]   groundAxisU_World_in
 *              First horizontal ground axis (matches buildFiniteWallSegment2d).
 * @param[in]   groundAxisV_World_in
 *              Second horizontal ground axis.
 * @param[in]   groundNormal_World_in
 *              Unit ground normal in the world frame.
 * @param[in]   endpointTrimRatio_in
 *              Forwarded to buildFiniteWallSegment2d.
 * @param[in]   minimumWallLength_m_in
 *              Forwarded to buildFiniteWallSegment2d.
 *
 * @return      True when a foreign room's own finite wall extent blocks
 *              the segment.
 */
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
