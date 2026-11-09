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
 * @file            moveFloor.cc
 *
 * @brief           Implements SemanticsManager::moveFloor(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <utility>

#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::moveFloor(unsigned long    sourceMapId_in,
                                unsigned long    destinationMapId_in,
                                semantic::Floor *p_floor_in)
{
    if (p_floor_in == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    std::unique_lock<std::shared_mutex> graphLock(graphMutex);
    const std::map<unsigned long, scene_graph::SceneGraph *>::iterator
        sourceIterator = sceneGraphs.find(sourceMapId_in);
    const std::map<unsigned long, scene_graph::SceneGraph *>::iterator
        destinationIterator = sceneGraphs.find(destinationMapId_in);
    if (sourceIterator == sceneGraphs.end() ||
        destinationIterator == sceneGraphs.end())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    std::unique_ptr<semantic::Floor> p_movedFloor{};
    for (std::pair<const int, std::unique_ptr<scene_graph::Building>>
             &buildingEntry : sourceIterator->second->buildings)
    {
        if (p_movedFloor == nullptr)
        {
            p_movedFloor =
                extractEntryByPointer(buildingEntry.second->floors, p_floor_in);
        }
    }
    if (p_movedFloor == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    insertFloor(*destinationIterator->second, std::move(p_movedFloor));
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
