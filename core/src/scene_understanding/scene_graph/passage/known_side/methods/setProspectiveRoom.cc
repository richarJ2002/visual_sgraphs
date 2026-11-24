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
 * @file            setProspectiveRoom.cc
 *
 * @brief           Implements Passage::setProspectiveRoom(), declared in
 *                  Semantic/Passage.h.
 */

#include "scene_understanding/scene_graph/passage/objects/PassageClass.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

PassageStatus
    Passage::setProspectiveRoom(vs_graphs::core::semantic::Room *p_room_in)
{
    /* The room owning the host (first) face is the near side. */
    std::vector<geometric::Plane *> roomWalls{};
    if (p_room_in != nullptr &&
        p_room_in->getWalls(roomWalls) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (!associateWalls.empty() &&
        std::find(roomWalls.begin(), roomWalls.end(), associateWalls.front()) !=
            roomWalls.end())
    {
        return PassageStatus::PASSAGE_STATUS_SUCCESS;
    }
    p_prospectiveRoom = p_room_in;

    return PassageStatus::PASSAGE_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
