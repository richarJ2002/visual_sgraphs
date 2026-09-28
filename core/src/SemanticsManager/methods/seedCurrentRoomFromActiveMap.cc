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

void SemanticsManager::seedCurrentRoomFromActiveMap(Map *p_activeMap_in)
{
    if (p_activeMap_in == nullptr)
    {
        return;
    }

    std::lock_guard<std::mutex> currentRoomLock(currentRoomMutex);
    if (currentRoomId != -1)
    {
        return;
    }

    const std::vector<semantic::Room *> rooms = p_activeMap_in->getAllRooms();
    for (semantic::Room *p_room : rooms)
    {
        bool roomIsBad{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        semantic::Room::RoomVariant roomVariant{};
        if ((p_room != nullptr && !roomIsBad) &&
            p_room->getRoomVariant(roomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getRoomVariant cannot fail; continue as before.
        }
        if (p_room != nullptr && !roomIsBad &&
            roomVariant == semantic::Room::RoomVariant::ROOM)
        {
            int roomId{};
            if (p_room->getId(roomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            currentRoomId = roomId;
            p_atlas->setCurrentSemanticRoomIdentity(currentRoomId);
            return;
        }
    }
}

} // namespace core
} // namespace vs_graphs
