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
 * @file            getMapRoomsInIdOrder.cc
 *
 * @brief           Implements SemanticsManager::getMapRoomsInIdOrder(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <vector>

#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::getMapRoomsInIdOrder(
    unsigned long                  mapId_in,
    std::vector<semantic::Room *> &rooms_out) const
{
    rooms_out.clear();
    GraphLockScope           graphScope{};
    scene_graph::SceneGraph *p_sceneGraph = nullptr;
    if (lockMapGraph(mapId_in,
                     GraphLockMode::GRAPH_LOCK_MODE_SHARED,
                     graphScope,
                     p_sceneGraph) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    /* Detected rooms first: on an id held by both lists the live detected
     * room is kept. Flagged rooms are hidden. */
    std::map<int, semantic::Room *> roomsById{};
    for (const std::pair<const int, std::unique_ptr<semantic::Room>>
             &roomEntry : p_sceneGraph->detectedRooms)
    {
        if (isRoomLive(roomEntry.second.get()))
        {
            roomsById.emplace(roomEntry.first, roomEntry.second.get());
        }
    }
    for (const std::pair<const int, std::unique_ptr<semantic::Room>>
             &roomEntry : p_sceneGraph->candidateRooms)
    {
        if (isRoomLive(roomEntry.second.get()))
        {
            roomsById.emplace(roomEntry.first, roomEntry.second.get());
        }
    }
    for (const std::pair<const int, semantic::Room *> &roomEntry : roomsById)
    {
        rooms_out.push_back(roomEntry.second);
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
