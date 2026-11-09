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
    /* Both graphs are held exclusive, locked in ascending map-id order, for
     * the whole move. */
    GraphLockScope graphScope{};
    if (lockGraphs({sourceMapId_in, destinationMapId_in},
                   GraphLockMode::GRAPH_LOCK_MODE_EXCLUSIVE,
                   graphScope) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }
    scene_graph::SceneGraph *p_sourceSceneGraph =
        findSceneGraph(sourceMapId_in);
    scene_graph::SceneGraph *p_destinationSceneGraph =
        findSceneGraph(destinationMapId_in);

    std::unique_ptr<semantic::Floor> p_movedFloor{};
    for (std::pair<const int, std::unique_ptr<scene_graph::Building>>
             &buildingEntry : p_sourceSceneGraph->buildings)
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

    insertFloor(*p_destinationSceneGraph, std::move(p_movedFloor));
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
