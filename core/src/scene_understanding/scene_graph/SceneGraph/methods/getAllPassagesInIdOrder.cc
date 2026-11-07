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
 * @file            getAllPassagesInIdOrder.cc
 *
 * @brief           Implements SceneGraph::getAllPassagesInIdOrder(), declared
 * in scene_graph/SceneGraph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphClass.h"

#include <map>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus SceneGraph::getAllPassagesInIdOrder(
    std::vector<semantic::Passage *> &passages_out) const
{
    passages_out.clear();
    const SceneMap        *p_activeSceneMap = nullptr;
    const SceneGraphStatus activeStatus = getActiveSceneMap(p_activeSceneMap);
    if (p_activeSceneMap != nullptr)
    {
        for (const std::pair<const int, semantic::Passage *> &passageEntry :
             p_activeSceneMap->passages)
        {
            passages_out.push_back(passageEntry.second);
        }
    }
    return activeStatus;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
