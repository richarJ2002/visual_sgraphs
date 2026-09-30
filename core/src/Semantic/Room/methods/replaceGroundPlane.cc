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
 * @file            replaceGroundPlane.cc
 *
 * @brief           Implements Room::replaceGroundPlane(), declared in
 *                  Semantic/Room.h.
 */

#include "Semantic/Room.h"
#include <algorithm>
#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::replaceGroundPlane(geometric::Plane *p_retiredGround_in,
                                    geometric::Plane *p_retainedGround_in,
                                    bool &wasGroundPlaneReplaced_out)
{
    if (p_retiredGround_in == nullptr || p_retainedGround_in == nullptr ||
        p_retiredGround_in == p_retainedGround_in)
    {
        return RoomStatus::ROOM_STATUS_INVALID_ARGUMENT;
    }

    std::lock_guard<std::mutex> lock(wallsMutex);

    if (p_groundPlane != p_retiredGround_in)
    {
        wasGroundPlaneReplaced_out = false;
        return RoomStatus::ROOM_STATUS_SUCCESS;
    }

    p_groundPlane              = p_retainedGround_in;
    wasGroundPlaneReplaced_out = true;
    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
