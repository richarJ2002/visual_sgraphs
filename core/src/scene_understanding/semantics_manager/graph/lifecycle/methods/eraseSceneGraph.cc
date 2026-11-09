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
 * @file            eraseSceneGraph.cc
 *
 * @brief           Implements SemanticsManager::eraseSceneGraph(), declared in
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

SemanticsManagerStatus SemanticsManager::eraseSceneGraph(unsigned long mapId_in)
{
    std::unique_lock<std::shared_mutex> graphLock(registryMutex);
    const std::map<unsigned long, scene_graph::SceneGraph *>::iterator
        sceneGraphIterator = sceneGraphs.find(mapId_in);
    if (sceneGraphIterator != sceneGraphs.end())
    {
        /* Other maps may still link to this map's rooms (their starting and
         * final rooms), so the nodes are retired and freed by a sweep after
         * those links are cleared. */
        retireSceneGraphContents(*sceneGraphIterator->second, retiredNodes);
        sceneGraphs.erase(sceneGraphIterator);
    }
    if (activeGraphMapId == mapId_in)
    {
        activeGraphMapId.reset();
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
