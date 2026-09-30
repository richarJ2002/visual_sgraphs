/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            addRoom.cc
 *
 * @brief           Implements Floor::addRoom(), declared in Semantic/Floor.h.
 */

#include "Semantic/Floor.h"
#include "Semantic/Room.h"
#include "Semantic/RoomStatus.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

FloorStatus Floor::addRoom(vs_graphs::core::semantic::Room *p_value_inout)
{
    if (p_value_inout == nullptr)
    {
        return FloorStatus::FLOOR_STATUS_SUCCESS;
    }

    Floor *p_previousFloor = nullptr;
    if (p_value_inout->getFloor(p_previousFloor) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getFloor returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_previousFloor != nullptr && p_previousFloor != this)
    {
        if (p_previousFloor->detachRoom(p_value_inout) !=
            FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: detachRoom returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    {
        std::lock_guard<std::mutex> lock(roomsMutex);
        const bool                  alreadyPresent =
            std::find(rooms.begin(), rooms.end(), p_value_inout) != rooms.end();

        if (!alreadyPresent)
        {
            rooms.push_back(p_value_inout);
        }
    }

    if (p_value_inout->setFloor(this) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setFloor returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    return FloorStatus::FLOOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
