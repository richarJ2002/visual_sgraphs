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
 * @file            eraseRoom.cc
 *
 * @brief           Implements SemanticsManager::eraseRoom(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::eraseRoom(unsigned long   mapId_in,
                                                   semantic::Room *p_room_in)
{
    std::unique_lock<std::shared_mutex> graphLock(graphMutex);
    std::map<unsigned long, scene_graph::SceneMap *>::iterator
        sceneMapIterator = sceneMaps.find(mapId_in);
    if (sceneMapIterator == sceneMaps.end())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    /* Retired by pointer, so a room whose id changed after it was added
     * still leaves the tree. */
    scene_graph::SceneMap &sceneMap = *sceneMapIterator->second;
    retireEntryByPointer(sceneMap.detectedRooms, p_room_in, retiredNodes.rooms);
    retireEntryByPointer(sceneMap.candidateRooms,
                         p_room_in,
                         retiredNodes.rooms);
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
