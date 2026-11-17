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

/*!
 * @file            adoptBootstrapRoom.cc
 *
 * @brief           Implements SemanticsManager::adoptBootstrapRoom(), declared
 *                  in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::adoptBootstrapRoom(map::Map       *p_activeMap_inout,
                                          semantic::Room *p_room_inout)
{
    int roomId{};
    if (p_room_inout->getId(roomId) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    {
        std::lock_guard<std::mutex> currentRoomLock(currentRoomMutex);
        currentRoomId = roomId;
        if (p_atlas->setCurrentSemanticRoomIdentity(roomId) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCurrentSemanticRoomIdentity returned a "
                         "failure status although it cannot fail; continuing "
                         "as before.",
                         __func__);
        }
    }

    /* The UAV starts inside the bootstrap room: presence evidences entry.
     * Marked outside the current-room lock; the room owns its mutex. */
    if (p_room_inout->setPreviouslyVisited(true) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPreviouslyVisited returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Mission-chain trace: the room this map started with. Set once; later
     * bootstrap cycles must not overwrite it. */
    int startingRoomId = map::Map::NO_LINKED_ROOM_ID;
    if (p_activeMap_inout->getStartingRoomId(startingRoomId) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getStartingRoomId returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (startingRoomId == map::Map::NO_LINKED_ROOM_ID &&
        p_activeMap_inout->setStartingRoomId(roomId) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setStartingRoomId returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
