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
 * @file            removeInvalidWalls.cc
 *
 * @brief           Implements Room::removeInvalidWalls(), declared in
 *                  Semantic/Room.h.
 */

#include "Geometric/Plane.h"
#include "Geometric/PlaneStatus.h"
#include "Semantic/Room.h"
#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::removeInvalidWalls(std::size_t &removedWallCount_out)
{
    std::lock_guard<std::mutex> lock(wallsMutex);

    walls.erase(std::remove_if(
                    walls.begin(),
                    walls.end(),
                    [](geometric::Plane *p_wall)
                    {
                        bool wallIsBad{};
                        if (!(p_wall == nullptr) &&
                            p_wall->isBad(wallIsBad) !=
                                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: isBad returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        return p_wall == nullptr || wallIsBad;
                    }),
                walls.end());

    removedWallCount_out = walls.size();
    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
