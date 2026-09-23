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

namespace vs_graphs
{
namespace core
{

semantic::Room *SemanticsManager::findRoomByMapAndId(long unsigned int mapId_in,
                                                     int roomId_in) const
{
    for (Map *p_map : p_atlas->getAllMaps())
    {
        if (p_map == nullptr || p_map->getId() != mapId_in)
        {
            continue;
        }
        for (semantic::Room *p_room : p_map->getAllRooms())
        {
            if (p_room != nullptr && !p_room->isBad() &&
                p_room->getRoomVariant() == semantic::Room::RoomVariant::ROOM &&
                p_room->getId() == roomId_in)
            {
                return p_room;
            }
        }
        break;
    }
    return nullptr;
}

} // namespace core
} // namespace vs_graphs
