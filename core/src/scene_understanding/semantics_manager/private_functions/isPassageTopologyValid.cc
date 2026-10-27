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
 * @file            isPassageTopologyValid.cc
 *
 * @brief           Implements isPassageTopologyValid(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "private_functions.h"

#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    isPassageTopologyValid(const std::vector<semantic::Room *> &wallRooms_in,
                           std::size_t     linkedRoomCount_in,
                           semantic::Room *p_prospectiveRoom_in,
                           bool           &isPassageTopologyValid_out,
                           std::string    &violationReason_out)
{
    isPassageTopologyValid_out = false;
    if (wallRooms_in.empty() || wallRooms_in.size() > 2U)
    {
        violationReason_out =
            "wall_count_" + std::to_string(wallRooms_in.size());
    }
    else if (wallRooms_in.size() == 2U && wallRooms_in[0] != nullptr &&
             wallRooms_in[0] == wallRooms_in[1])
    {
        violationReason_out = "walls_in_same_room";
    }
    else if (linkedRoomCount_in > 2U)
    {
        violationReason_out =
            "linked_room_count_" + std::to_string(linkedRoomCount_in);
    }
    else if (p_prospectiveRoom_in != nullptr &&
             p_prospectiveRoom_in == wallRooms_in[0])
    {
        violationReason_out = "prospective_room_is_host_room";
    }
    else
    {
        violationReason_out.clear();
        isPassageTopologyValid_out = true;
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
