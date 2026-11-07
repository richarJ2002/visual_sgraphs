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
 * @file            getPassageById.cc
 *
 * @brief           Implements SceneGraph::getPassageById(), declared in
 *                  scene_graph/SceneGraph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphClass.h"

#include <map>

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus
    SceneGraph::getPassageById(int                 passageId_in,
                               semantic::Passage *&p_passage_out) const
{
    p_passage_out                           = nullptr;
    const SceneMap        *p_activeSceneMap = nullptr;
    const SceneGraphStatus activeStatus = getActiveSceneMap(p_activeSceneMap);
    if (p_activeSceneMap != nullptr)
    {
        std::map<int, semantic::Passage *>::const_iterator passageIterator =
            p_activeSceneMap->passages.find(passageId_in);
        if (passageIterator != p_activeSceneMap->passages.end())
        {
            p_passage_out = passageIterator->second;
        }
    }
    return activeStatus;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
