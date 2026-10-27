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
 * @file            getIsSeparatedFrom.cc
 *
 * @brief           Implements Room::getIsSeparatedFrom(), declared in
 *                  semantic/room/objects/RoomClass.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantic/room/objects/RoomClass.h"
#include <algorithm>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::getIsSeparatedFrom(const Room *p_other_in,
                                    bool       &isSeparated_out) const
{
    isSeparated_out = false;
    if (p_other_in == nullptr || p_other_in == this)
    {
        return RoomStatus::ROOM_STATUS_SUCCESS;
    }

    std::vector<Passage *>          ownPassages{}, otherPassages{};
    std::vector<geometric::Plane *> ownWalls{}, otherWalls{};
    if (getPassages(ownPassages) != RoomStatus::ROOM_STATUS_SUCCESS ||
        p_other_in->getPassages(otherPassages) !=
            RoomStatus::ROOM_STATUS_SUCCESS ||
        getWalls(ownWalls) != RoomStatus::ROOM_STATUS_SUCCESS ||
        p_other_in->getWalls(otherWalls) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPassages/getWalls returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* A passage links at most two rooms, one per side. */
    for (Passage *p_passage : ownPassages)
    {
        if (p_passage != nullptr &&
            std::find(otherPassages.begin(), otherPassages.end(), p_passage) !=
                otherPassages.end())
        {
            isSeparated_out = true;
            return RoomStatus::ROOM_STATUS_SUCCESS;
        }
    }

    /* The two faces of one physical wall bound two different rooms. */
    for (geometric::Plane *p_wall : ownWalls)
    {
        geometric::Plane *p_twinFace = nullptr;
        if (p_wall != nullptr &&
            p_wall->getTwinFace(p_twinFace) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTwinFace returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_twinFace != nullptr &&
            std::find(otherWalls.begin(), otherWalls.end(), p_twinFace) !=
                otherWalls.end())
        {
            isSeparated_out = true;
            return RoomStatus::ROOM_STATUS_SUCCESS;
        }
    }
    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
