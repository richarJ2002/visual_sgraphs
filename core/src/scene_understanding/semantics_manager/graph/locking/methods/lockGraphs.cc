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
 * @file            lockGraphs.cc
 *
 * @brief           Implements SemanticsManager::lockGraphs(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <map>
#include <vector>

#include "scene_understanding/semantics_manager/graph/locking/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::lockGraphs(const std::vector<unsigned long> &mapIds_in,
                                 GraphLockMode                     mode_in,
                                 GraphLockScope &scope_out) const
{
    /* The map orders the graphs by id whatever the order of the request. */
    std::map<unsigned long, scene_graph::SceneGraph *> graphsInIdOrder{};
    for (const unsigned long mapId : mapIds_in)
    {
        scene_graph::SceneGraph *p_graph = findSceneGraph(mapId);
        if (p_graph == nullptr)
        {
            return SemanticsManagerStatus::
                SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
        }
        graphsInIdOrder[mapId] = p_graph;
    }
    if (!acquireGraphLocks(graphsInIdOrder, mode_in, scope_out))
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_BUSY;
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
