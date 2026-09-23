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

    std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
    if (currentRoomId_ != -1)
    {
        return;
    }

    const std::vector<semantic::Room *> rooms = p_activeMap_in->getAllRooms();
    for (semantic::Room *p_room : rooms)
    {
        if (p_room != nullptr && !p_room->isBad() &&
            p_room->getRoomVariant() == semantic::Room::RoomVariant::ROOM)
        {
            currentRoomId_ = p_room->getId();
            p_atlas->setCurrentSemanticRoomIdentity(currentRoomId_);
            return;
        }
    }
}

} // namespace core
} // namespace vs_graphs
