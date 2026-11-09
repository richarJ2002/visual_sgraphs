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
 * @file            getMapRoomsByIds.cc
 *
 * @brief           Implements SemanticsManager::getMapRoomsByIds(), declared
 *                  in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::getMapRoomsByIds(
    unsigned long                  mapId_in,
    const std::vector<int>        &roomIds_in,
    std::vector<semantic::Room *> &rooms_out) const
{
    rooms_out.clear();

    /* One shared scope for the whole list; each lookup re-enters it. */
    GraphLockScope graphScope{};
    if (lockGraphs({mapId_in},
                   GraphLockMode::GRAPH_LOCK_MODE_SHARED,
                   graphScope) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    for (int roomId : roomIds_in)
    {
        semantic::Room *p_room = nullptr;
        if ((getMapRoomById(mapId_in, roomId, p_room) ==
             SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS) &&
            (p_room != nullptr))
        {
            rooms_out.push_back(p_room);
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
