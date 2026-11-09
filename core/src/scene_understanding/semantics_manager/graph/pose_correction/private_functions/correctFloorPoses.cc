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
 * @file            correctFloorPoses.cc
 *
 * @brief           Implements correctFloorPoses(), declared in
 *                  semantics_manager/graph/pose_correction/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/pose_correction/private_functions/private_functions.h"

#include "map_state/map/map.h"
#include "mapping/geometric/Plane/Plane.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void correctFloorPoses(const PoseCorrectionField &field_in,
                       const map::Map            *p_map_in,
                       const std::map<semantic::Room *, g2o::Sim3>
                           &roomCorrections_oldWorldToNewWorld_in,
                       const std::map<semantic::Room *, Eigen::Vector3d>
                           &roomCentroids_oldWorld_m_in,
                       std::vector<semantic::Floor *> &floors_inout)
{
    for (semantic::Floor *p_floor : floors_inout)
    {
        if (p_floor == nullptr)
        {
            continue;
        }

        Eigen::Vector3d floorCentroid_oldWorld_m{};
        if (p_floor->getCentroid(floorCentroid_oldWorld_m) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        const g2o::Sim3 *p_floorCorrection = nullptr;
        double           closestRoomSquaredDistance_m2 =
            std::numeric_limits<double>::infinity();

        std::vector<semantic::Room *> floorRooms{};
        if (p_floor->getRooms(floorRooms) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRooms returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (semantic::Room *p_room : floorRooms)
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
            map::Map *p_roomMap = nullptr;
            if (!(p_room == nullptr || roomIsBad) &&
                p_room->getMap(p_roomMap) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room == nullptr || roomIsBad || p_roomMap != p_map_in)
            {
                continue;
            }

            const std::map<semantic::Room *, g2o::Sim3>::const_iterator
                correctionIterator =
                    roomCorrections_oldWorldToNewWorld_in.find(p_room);
            const std::map<semantic::Room *, Eigen::Vector3d>::const_iterator
                centroidIterator = roomCentroids_oldWorld_m_in.find(p_room);

            if (correctionIterator ==
                    roomCorrections_oldWorldToNewWorld_in.end() ||
                centroidIterator == roomCentroids_oldWorld_m_in.end())
            {
                continue;
            }

            const double squaredDistance_m2 =
                (centroidIterator->second - floorCentroid_oldWorld_m)
                    .squaredNorm();

            if (squaredDistance_m2 < closestRoomSquaredDistance_m2)
            {
                closestRoomSquaredDistance_m2 = squaredDistance_m2;
                p_floorCorrection             = &correctionIterator->second;
            }
        }

        if (p_floor->applyTransform(p_floorCorrection != nullptr
                                        ? *p_floorCorrection
                                        : field_in.selectCorrectionForPoint(
                                              floorCentroid_oldWorld_m)) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyTransform returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
