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

#include "private_functions.h"

namespace vs_graphs
{
namespace core
{

bool sharesRoomNameTag(Map *p_firstMap_in, Map *p_secondMap_in)
{
    std::unordered_set<std::string> firstMapRoomTags;
    for (semantic::Room *p_room : p_firstMap_in->getAllDetectedMapRooms())
    {
        if (p_room->hasRoomTag())
        {
            firstMapRoomTags.insert(p_room->getRoomTag());
        }
    }
    for (semantic::Room *p_room : p_firstMap_in->getAllMarkerBasedMapRooms())
    {
        if (p_room->hasRoomTag())
        {
            firstMapRoomTags.insert(p_room->getRoomTag());
        }
    }

    if (firstMapRoomTags.empty())
    {
        std::cout << "[SemMgr] sharesRoomNameTag: first map (id="
                  << p_firstMap_in->getId()
                  << ") has NO tagged rooms (detected="
                  << p_firstMap_in->getAllDetectedMapRooms().size()
                  << ", marker="
                  << p_firstMap_in->getAllMarkerBasedMapRooms().size() << ")"
                  << std::endl;
        return false;
    }

    for (semantic::Room *p_room : p_secondMap_in->getAllDetectedMapRooms())
    {
        if (p_room->hasRoomTag() &&
            firstMapRoomTags.count(p_room->getRoomTag()) != 0U)
        {
            std::cout << "[SemMgr] sharesRoomNameTag: MATCH found tag "
                      << p_room->getRoomTag() << " between maps "
                      << p_firstMap_in->getId() << " and "
                      << p_secondMap_in->getId() << std::endl;
            return true;
        }
    }
    for (semantic::Room *p_room : p_secondMap_in->getAllMarkerBasedMapRooms())
    {
        if (p_room->hasRoomTag() &&
            firstMapRoomTags.count(p_room->getRoomTag()) != 0U)
        {
            std::cout << "[SemMgr] sharesRoomNameTag: MATCH found tag "
                      << p_room->getRoomTag() << " between maps "
                      << p_firstMap_in->getId() << " and "
                      << p_secondMap_in->getId() << std::endl;
            return true;
        }
    }

    std::cout << "[SemMgr] sharesRoomNameTag: NO match. First map (id="
              << p_firstMap_in->getId() << ") tags: " << firstMapRoomTags.size()
              << ", second map (id=" << p_secondMap_in->getId() << ") detected="
              << p_secondMap_in->getAllDetectedMapRooms().size() << " marker="
              << p_secondMap_in->getAllMarkerBasedMapRooms().size()
              << std::endl;

    return false;
}

} // namespace core
} // namespace vs_graphs
