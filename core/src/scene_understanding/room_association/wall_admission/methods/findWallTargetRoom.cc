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
 * @file            findWallTargetRoom.cc
 *
 * @brief           Implements SemanticsManager::findWallTargetRoom(), declared
 *                  in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <rclcpp/logging.hpp>
#include <vector>

#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/room/Room.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::findWallTargetRoom(
    geometric::Plane                    *p_wall_in,
    const std::vector<semantic::Room *> &rooms_in,
    semantic::Room                      *p_currentRoom_in,
    int                                 &observerRoomId_out,
    semantic::Room                     *&p_targetRoom_out) const
{
    p_targetRoom_out = p_currentRoom_in;
    std::vector<semantic::Passage *> passages{};
    if (getAllPassagesInIdOrder(passages) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        findRedirectedObserverRoomId(p_wall_in,
                                     rooms_in,
                                     passages,
                                     observerRoomId_out) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        (observerRoomId_out >= 0 &&
         getRoomById(observerRoomId_out, p_targetRoom_out) !=
             SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassagesInIdOrder/findRedirectedObserverRoomId/"
                     "getRoomById returned a failure status; continuing as "
                     "before.",
                     __func__);
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
