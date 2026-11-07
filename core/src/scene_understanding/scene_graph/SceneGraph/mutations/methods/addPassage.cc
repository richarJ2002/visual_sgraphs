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

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <shared_mutex>

#include "scene_understanding/scene_graph/Passage/Passage.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus SceneGraph::addPassage(unsigned long      mapId_in,
                                        semantic::Passage *p_passage_in)
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
        return SceneGraphStatus::SCENE_GRAPH_STATUS_MAP_NOT_FOUND;
    }

    int passageId{};
    if (p_passage_in->getId(passageId) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* A negative id, or an id another passage holds, is rejected and left
     * for the caller to resolve. */
    std::map<int, semantic::Passage *> &passagesById =
        sceneMapIterator->second->passages;
    const std::map<int, semantic::Passage *>::iterator existingPassage =
        passagesById.find(passageId);
    if (passageId < 0 || (existingPassage != passagesById.end() &&
                          existingPassage->second != p_passage_in))
    {
        std::cerr << "[Map] Passage ID collision for " << passageId
                  << "; caller must resolve it before destination insertion."
                  << std::endl;
        return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
    }
    passagesById.insert_or_assign(passageId, p_passage_in);
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
