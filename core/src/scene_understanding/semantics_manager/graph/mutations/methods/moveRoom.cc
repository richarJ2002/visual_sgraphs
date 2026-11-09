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
 * @file            moveRoom.cc
 *
 * @brief           Implements SemanticsManager::moveRoom(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <utility>

#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::moveRoom(unsigned long   sourceMapId_in,
                               unsigned long   destinationMapId_in,
                               semantic::Room *p_room_in,
                               RoomList        destinationList_in)
{
    if (p_room_in == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    std::unique_lock<std::shared_mutex> graphLock(registryMutex);
    const std::map<unsigned long, scene_graph::SceneGraph *>::iterator
        sourceIterator = sceneGraphs.find(sourceMapId_in);
    const std::map<unsigned long, scene_graph::SceneGraph *>::iterator
        destinationIterator = sceneGraphs.find(destinationMapId_in);
    if (sourceIterator == sceneGraphs.end() ||
        destinationIterator == sceneGraphs.end())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    /* The room is taken out by pointer, so it moves under the id it holds
     * now, which the caller may have changed. */
    scene_graph::SceneGraph        &sourceMap = *sourceIterator->second;
    std::unique_ptr<semantic::Room> p_movedRoom =
        extractEntryByPointer(sourceMap.detectedRooms, p_room_in);
    if (p_movedRoom == nullptr)
    {
        p_movedRoom =
            extractEntryByPointer(sourceMap.candidateRooms, p_room_in);
    }
    if (p_movedRoom == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    scene_graph::SceneGraph &destinationMap = *destinationIterator->second;
    insertRoom(destinationList_in == RoomList::ROOM_LIST_DETECTED
                   ? destinationMap.detectedRooms
                   : destinationMap.candidateRooms,
               std::move(p_movedRoom),
               destinationMapId_in,
               retiredNodes);
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
