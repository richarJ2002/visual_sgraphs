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
 * @file            correctRoomPoses.cc
 *
 * @brief           Implements correctRoomPoses(), declared in
 *                  semantics_manager/graph/pose_correction/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/pose_correction/private_functions/private_functions.h"

#include "mapping/geometric/Plane/Plane.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include <limits>
#include <rclcpp/logging.hpp>
#include <set>

namespace vs_graphs
{
namespace core
{

void correctRoomPoses(
    const PoseCorrectionField     &field_in,
    std::vector<semantic::Room *> &rooms_inout,
    std::map<semantic::Room *, g2o::Sim3>
        &roomCorrections_oldWorldToNewWorld_out,
    std::map<semantic::Room *, Eigen::Vector3d> &roomCentroids_oldWorld_m_out)
{
    std::set<semantic::Room *> correctedRooms;

    for (semantic::Room *p_room : rooms_inout)
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
            !correctedRooms.insert(p_room).second)
        {
            continue;
        }

        Eigen::Vector3d roomCentroid_oldWorld_m{};
        if (p_room->getCentroid(roomCentroid_oldWorld_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        roomCentroids_oldWorld_m_out.insert_or_assign(p_room,
                                                      roomCentroid_oldWorld_m);

        const g2o::Sim3 *p_roomCorrection = nullptr;
        double           closestWallSquaredDistance_m2 =
            std::numeric_limits<double>::infinity();

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
            const std::map<geometric::Plane *, g2o::Sim3>::const_iterator
                correctionIterator =
                    field_in.planeCorrections_oldWorldToNewWorld.find(p_wall);
            const std::map<geometric::Plane *, Eigen::Vector3d>::const_iterator
                centroidIterator =
                    field_in.planeCentroids_oldWorld_m.find(p_wall);

            if (correctionIterator ==
                    field_in.planeCorrections_oldWorldToNewWorld.end() ||
                centroidIterator == field_in.planeCentroids_oldWorld_m.end())
            {
                continue;
            }

            const double squaredDistance_m2 =
                (centroidIterator->second - roomCentroid_oldWorld_m)
                    .squaredNorm();

            if (squaredDistance_m2 < closestWallSquaredDistance_m2)
            {
                closestWallSquaredDistance_m2 = squaredDistance_m2;
                p_roomCorrection              = &correctionIterator->second;
            }
        }

        const g2o::Sim3 &roomCorrection_oldWorldToNewWorld =
            p_roomCorrection != nullptr
                ? *p_roomCorrection
                : field_in.selectCorrectionForPoint(roomCentroid_oldWorld_m);

        if (p_room->applyTransform(roomCorrection_oldWorldToNewWorld) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyTransform returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        roomCorrections_oldWorldToNewWorld_out.insert_or_assign(
            p_room,
            roomCorrection_oldWorldToNewWorld);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
