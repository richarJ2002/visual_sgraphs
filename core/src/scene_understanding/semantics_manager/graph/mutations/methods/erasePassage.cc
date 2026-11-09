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
 * @file            erasePassage.cc
 *
 * @brief           Implements SemanticsManager::erasePassage(), declared in
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
    SemanticsManager::erasePassage(unsigned long      mapId_in,
                                   semantic::Passage *p_passage_in)
{
    GraphLockScope           graphScope{};
    scene_graph::SceneGraph *p_sceneGraph = nullptr;
    if (lockMapGraph(mapId_in,
                     GraphLockMode::GRAPH_LOCK_MODE_EXCLUSIVE,
                     graphScope,
                     p_sceneGraph) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    scene_graph::SceneGraph &sceneGraph = *p_sceneGraph;
    for (std::pair<const int, std::unique_ptr<scene_graph::Building>>
             &buildingEntry : sceneGraph.buildings)
    {
        eraseEntriesByPointer(buildingEntry.second->passages, p_passage_in);
    }
    retireEntryByPointer(sceneGraph.passages,
                         p_passage_in,
                         p_sceneGraph->retiredNodes.passages);
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
