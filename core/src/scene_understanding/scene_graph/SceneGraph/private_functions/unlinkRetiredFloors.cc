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
 * @file            unlinkRetiredFloors.cc
 *
 * @brief           Implements unlinkRetiredFloors(), declared in
 *                  scene_graph/SceneGraph/private_functions/private_functions.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/private_functions/private_functions.h"

#include <memory>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

void unlinkRetiredFloors(
    const std::vector<std::unique_ptr<semantic::Floor>> &retiredFloors_in,
    const std::vector<semantic::Room *>                 &liveRooms_in)
{
    for (const std::unique_ptr<semantic::Floor> &p_retiredFloor :
         retiredFloors_in)
    {
        for (semantic::Room *p_room : liveRooms_in)
        {
            semantic::Floor *p_roomFloor = nullptr;
            if (p_room->getFloor(p_roomFloor) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getFloor returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_roomFloor == p_retiredFloor.get() &&
                p_room->setFloor(nullptr) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setFloor returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
