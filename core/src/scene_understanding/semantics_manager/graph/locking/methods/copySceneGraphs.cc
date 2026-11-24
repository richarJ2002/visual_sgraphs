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
 * @file            copySceneGraphs.cc
 *
 * @brief           Implements SemanticsManager::copySceneGraphs(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <map>
#include <mutex>
#include <shared_mutex>

namespace vs_graphs
{
namespace core
{

std::map<unsigned long, scene_graph::SceneGraph *>
    SemanticsManager::copySceneGraphs() const
{
    std::shared_lock<std::shared_mutex> registryLock(registryMutex);
    return sceneGraphs;
}

} /* namespace core */
} /* namespace vs_graphs */
