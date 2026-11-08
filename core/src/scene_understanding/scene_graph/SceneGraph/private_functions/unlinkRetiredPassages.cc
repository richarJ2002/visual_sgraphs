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
 * @file            unlinkRetiredPassages.cc
 *
 * @brief           Implements unlinkRetiredPassages(), declared in
 *                  scene_graph/SceneGraph/private_functions/private_functions.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/private_functions/private_functions.h"

#include <memory>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

void unlinkRetiredPassages(
    const std::vector<std::unique_ptr<semantic::Passage>> &retiredPassages_in,
    const std::vector<semantic::Room *>                   &liveRooms_in)
{
    for (const std::unique_ptr<semantic::Passage> &p_retiredPassage :
         retiredPassages_in)
    {
        for (semantic::Room *p_room : liveRooms_in)
        {
            bool wasPassageRemoved{};
            if (p_room->removePassageAssociation(p_retiredPassage.get(),
                                                 wasPassageRemoved) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: removePassageAssociation rejected its "
                             "input; continuing as before.",
                             __func__);
            }
        }
    }
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
