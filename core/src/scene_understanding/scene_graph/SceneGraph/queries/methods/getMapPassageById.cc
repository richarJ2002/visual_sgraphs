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
 * @file            getMapPassageById.cc
 *
 * @brief           Implements SceneGraph::getMapPassageById(), declared in
 *                  scene_graph/SceneGraph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphClass.h"

#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include "scene_understanding/scene_graph/SceneGraph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus
    SceneGraph::getMapPassageById(unsigned long       mapId_in,
                                  int                 passageId_in,
                                  semantic::Passage *&p_passage_out) const
{
    p_passage_out = nullptr;
    std::shared_lock<std::shared_mutex> graphLock(graphMutex);
    std::map<unsigned long, std::unique_ptr<SceneMap>>::const_iterator
        sceneMapIterator = sceneMaps.find(mapId_in);
    if (sceneMapIterator == sceneMaps.end())
    {
        return SceneGraphStatus::SCENE_GRAPH_STATUS_MAP_NOT_FOUND;
    }

    std::map<int, std::unique_ptr<semantic::Passage>>::const_iterator
        passageIterator = sceneMapIterator->second->passages.find(passageId_in);
    if (passageIterator != sceneMapIterator->second->passages.end() &&
        isPassageLive(passageIterator->second.get()))
    {
        p_passage_out = passageIterator->second.get();
    }
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
