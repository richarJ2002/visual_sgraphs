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

/*!
 * @file            lockMapGraph.cc
 *
 * @brief           Implements SemanticsManager::lockMapGraph(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <map>

#include "scene_understanding/semantics_manager/graph/locking/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::lockMapGraph(
    unsigned long             mapId_in,
    GraphLockMode             mode_in,
    GraphLockScope           &scope_out,
    scene_graph::SceneGraph *&p_sceneGraph_out) const
{
    p_sceneGraph_out = findSceneGraph(mapId_in);
    if (p_sceneGraph_out == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }
    std::map<unsigned long, scene_graph::SceneGraph *> graphInIdOrder{
        {mapId_in, p_sceneGraph_out}};
    if (!acquireGraphLocks(graphInIdOrder, mode_in, scope_out))
    {
        p_sceneGraph_out = nullptr;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_BUSY;
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
