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
 * @file            flagRoomBad.cc
 *
 * @brief           Implements flagRoomBad(), declared in
 *                  semantics_manager/graph/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{

void flagRoomBad(const scene_graph::SceneMap &sceneMap_in,
                 semantic::Room              *p_room_inout)
{
    if (p_room_inout == nullptr)
    {
        return;
    }

    std::vector<semantic::Passage *> roomPassages{};
    if (p_room_inout->getPassages(roomPassages) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_inout->setBad() != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPassages/setBad returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }

    /* The room is flagged first, so it no longer counts as a live end of its
     * own passages. A passage left without any live room goes with it. */
    for (semantic::Passage *p_passage : roomPassages)
    {
        if (isPassageLive(p_passage) &&
            !hasLiveRoomLink(sceneMap_in, p_passage) &&
            p_passage->setBad() !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
