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
 * @file            findWallRoom.cc
 *
 * @brief           Implements findWallRoom(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "private_functions.h"

#include <algorithm>
#include <vector>

#include <rclcpp/logging.hpp>

#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    findWallRoom(const std::vector<semantic::Room *> &rooms_in,
                 geometric::Plane                    *p_wall_in,
                 semantic::Room                     *&p_wallRoom_out)
{
    p_wallRoom_out = nullptr;
    if (p_wall_in == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    for (semantic::Room *p_room : rooms_in)
    {
        bool roomIsBad = true;
        if (p_room != nullptr && p_room->isBad(roomIsBad) !=
                                     semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || roomIsBad)
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
        if (std::find(roomWalls.begin(), roomWalls.end(), p_wall_in) !=
            roomWalls.end())
        {
            p_wallRoom_out = p_room;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
