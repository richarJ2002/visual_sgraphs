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

#include "SemanticsManager.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::findRoomByMapAndId(long unsigned int mapId_in,
                                         int               roomId_in,
                                         semantic::Room  *&p_room_out) const
{
    std::vector<Map *> atlasAllMaps{};
    if (p_atlas->getAllMaps(atlasAllMaps) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (Map *p_map : atlasAllMaps)
    {
        unsigned long mapId{};
        if (!(p_map == nullptr) &&
            p_map->getId(mapId) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_map == nullptr || mapId != mapId_in)
        {
            continue;
        }
        std::vector<semantic::Room *> mapAllRooms{};
        if (p_map->getAllRooms(mapAllRooms) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllRooms returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (semantic::Room *p_room : mapAllRooms)
        {
            bool roomIsBad{};
            if ((p_room != nullptr) &&
                p_room->isBad(roomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Room::RoomVariant roomVariant{};
            if ((p_room != nullptr && !roomIsBad) &&
                p_room->getRoomVariant(roomVariant) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomVariant returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int roomId{};
            if ((p_room != nullptr && !roomIsBad &&
                 roomVariant == semantic::Room::RoomVariant::ROOM) &&
                p_room->getId(roomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room != nullptr && !roomIsBad &&
                roomVariant == semantic::Room::RoomVariant::ROOM &&
                roomId == roomId_in)
            {
                p_room_out = p_room;
                return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
            }
        }
        break;
    }
    p_room_out = nullptr;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
