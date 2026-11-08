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
 * @file            isRoomLive.cc
 *
 * @brief           Implements isRoomLive(), declared in
 *                  scene_graph/scene_graph/private_functions/private_functions.h.
 */

#include "scene_understanding/scene_graph/scene_graph/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

bool isRoomLive(semantic::Room *p_room_in)
{
    if (p_room_in == nullptr)
    {
        return false;
    }
    bool roomIsBad{};
    if (p_room_in->isBad(roomIsBad) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    return !roomIsBad;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
