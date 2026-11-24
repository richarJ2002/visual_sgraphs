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
 * @file            findLiveRoomById.cc
 *
 * @brief           Implements findLiveRoomById(), declared in
 *                  bootstrap/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/bootstrap/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

semantic::Room *findLiveRoomById(const std::vector<semantic::Room *> &rooms_in,
                                 int                                  roomId_in)
{
    if (roomId_in < 0)
    {
        return nullptr;
    }
    for (semantic::Room *p_room : rooms_in)
    {
        if (!isLiveConfirmedRoom(p_room))
        {
            continue;
        }
        int roomId{};
        if (p_room->getId(roomId) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (roomId == roomId_in)
        {
            return p_room;
        }
    }
    return nullptr;
}

} /* namespace core */
} /* namespace vs_graphs */
