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
 * @file            getMapRoomById.cc
 *
 * @brief           Implements SemanticsManager::getMapRoomById(), declared in
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

SemanticsManagerStatus
    SemanticsManager::getMapRoomById(unsigned long    mapId_in,
                                     int              roomId_in,
                                     semantic::Room *&p_room_out) const
{
    p_room_out = nullptr;
    std::shared_lock<std::shared_mutex> graphLock(registryMutex);
    std::map<unsigned long, scene_graph::SceneGraph *>::const_iterator
        sceneGraphIterator = sceneGraphs.find(mapId_in);
    if (sceneGraphIterator == sceneGraphs.end())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    /* Detected rooms first, as getMapRoomsInIdOrder() keeps them; a flagged
     * room is hidden and lets the other list answer. */
    const scene_graph::SceneGraph &sceneGraph = *sceneGraphIterator->second;
    std::map<int, std::unique_ptr<semantic::Room>>::const_iterator
        roomIterator = sceneGraph.detectedRooms.find(roomId_in);
    if (roomIterator != sceneGraph.detectedRooms.end() &&
        isRoomLive(roomIterator->second.get()))
    {
        p_room_out = roomIterator->second.get();
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    roomIterator = sceneGraph.candidateRooms.find(roomId_in);
    if (roomIterator != sceneGraph.candidateRooms.end() &&
        isRoomLive(roomIterator->second.get()))
    {
        p_room_out = roomIterator->second.get();
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
