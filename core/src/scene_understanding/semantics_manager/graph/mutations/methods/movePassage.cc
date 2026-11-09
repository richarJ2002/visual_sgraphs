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
 * @file            movePassage.cc
 *
 * @brief           Implements SemanticsManager::movePassage(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <utility>

#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::movePassage(unsigned long      sourceMapId_in,
                                  unsigned long      destinationMapId_in,
                                  semantic::Passage *p_passage_in)
{
    if (p_passage_in == nullptr)
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

    /* The passage is taken out by pointer, so it moves under the id it holds
     * now, which the caller may have changed. */
    scene_graph::SceneGraph           &sourceMap = *p_sourceSceneGraph;
    std::unique_ptr<semantic::Passage> p_movedPassage =
        extractEntryByPointer(sourceMap.passages, p_passage_in);
    if (p_movedPassage == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    insertPassage(*p_destinationSceneGraph,
                  std::move(p_movedPassage),
                  p_destinationSceneGraph->retiredNodes);
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
