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
 * @file            isLiveConfirmedRoom.cc
 *
 * @brief           Implements isLiveConfirmedRoom(), declared in
 *                  bootstrap/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/bootstrap/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

bool isLiveConfirmedRoom(semantic::Room *p_room_in)
{
    if (p_room_in == nullptr)
    {
        return false;
    }
    bool                        roomIsBad{};
    semantic::Room::RoomVariant roomVariant{};
    if (p_room_in->isBad(roomIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_in->getRoomVariant(roomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: a room getter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    return !roomIsBad && roomVariant == semantic::Room::RoomVariant::ROOM;
}

} /* namespace core */
} /* namespace vs_graphs */
