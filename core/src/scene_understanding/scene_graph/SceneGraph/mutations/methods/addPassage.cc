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
 * @file            addPassage.cc
 *
 * @brief           Implements SceneGraph::addPassage(), declared in
 *                  scene_graph/SceneGraph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphClass.h"

#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <utility>

#include "scene_understanding/scene_graph/Passage/Passage.h"
#include "scene_understanding/scene_graph/SceneGraph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus
    SceneGraph::addPassage(unsigned long                      mapId_in,
                           std::unique_ptr<semantic::Passage> p_passage_in)
{
    if (p_passage_in == nullptr)
    {
        return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
    }
    std::unique_lock<std::shared_mutex> graphLock(graphMutex);
    std::map<unsigned long, std::unique_ptr<SceneMap>>::iterator
        sceneMapIterator = sceneMaps.find(mapId_in);
    if (sceneMapIterator == sceneMaps.end())
    {
        retiredNodes.passages.push_back(std::move(p_passage_in));
        return SceneGraphStatus::SCENE_GRAPH_STATUS_MAP_NOT_FOUND;
    }

    insertPassage(*sceneMapIterator->second,
                  std::move(p_passage_in),
                  retiredNodes);
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
