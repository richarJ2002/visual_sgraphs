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
 * @file            sweepBadNodes.cc
 *
 * @brief           Implements SemanticsManager::sweepBadNodes(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <shared_mutex>
#include <utility>
#include <vector>

#include "scene_understanding/semantics_manager/graph/locking/private_functions/private_functions.h"
#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::sweepBadNodes()
{
    /* Dropped maps leave the registry for good; their graphs are locked and
     * emptied below, together with the registered ones. */
    std::map<unsigned long, scene_graph::SceneGraph *> graphsInIdOrder =
        copySceneGraphs();
    std::vector<scene_graph::SceneGraph *> erasedGraphs{};
    {
        std::unique_lock<std::shared_mutex> registryLock(registryMutex);
        erasedGraphs = std::move(erasedSceneGraphs);
        erasedSceneGraphs.clear();
    }

    /* Every graph is held exclusive from the first unlink to the last free, so
     * no reader can see a node between the two. Lock order: graph locks in
     * ascending map id, then dropped maps (nobody else locks those). */
    GraphLockScope graphScope{};
    if (!acquireGraphLocks(graphsInIdOrder,
                           GraphLockMode::GRAPH_LOCK_MODE_EXCLUSIVE,
                           graphScope))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: an exclusive lock cannot fail to be granted.",
                     __func__);
    }
    std::map<unsigned long, scene_graph::SceneGraph *> erasedInIdOrder{};
    unsigned long                                      erasedKey = 0UL;
    for (scene_graph::SceneGraph *p_erasedGraph : erasedGraphs)
    {
        erasedInIdOrder[erasedKey++] = p_erasedGraph;
    }
    if (!acquireGraphLocks(erasedInIdOrder,
                           GraphLockMode::GRAPH_LOCK_MODE_EXCLUSIVE,
                           graphScope))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: an exclusive lock cannot fail to be granted.",
                     __func__);
    }

    /* The batch is declared after the scope, so it is freed while the locks
     * are still held. */
    scene_graph::RetiredNodes retiredBatch{};
    for (scene_graph::SceneGraph *p_erasedGraph : erasedGraphs)
    {
        retireSceneGraphContents(*p_erasedGraph, p_erasedGraph->retiredNodes);
    }
    std::vector<scene_graph::SceneGraph *> allGraphs{};
    for (std::pair<const unsigned long, scene_graph::SceneGraph *>
             &sceneGraphEntry : graphsInIdOrder)
    {
        retireBadNodes(*sceneGraphEntry.second,
                       sceneGraphEntry.second->retiredNodes);
        allGraphs.push_back(sceneGraphEntry.second);
    }
    allGraphs.insert(allGraphs.end(), erasedGraphs.begin(), erasedGraphs.end());
    for (scene_graph::SceneGraph *p_graph : allGraphs)
    {
        for (std::unique_ptr<semantic::Room> &p_room :
             p_graph->retiredNodes.rooms)
        {
            retiredBatch.rooms.push_back(std::move(p_room));
        }
        for (std::unique_ptr<semantic::Passage> &p_passage :
             p_graph->retiredNodes.passages)
        {
            retiredBatch.passages.push_back(std::move(p_passage));
        }
        for (std::unique_ptr<semantic::Floor> &p_floor :
             p_graph->retiredNodes.floors)
        {
            retiredBatch.floors.push_back(std::move(p_floor));
        }
        p_graph->retiredNodes = scene_graph::RetiredNodes{};
    }

    /* The maps' starting and final room links are room ids: a retired room's
     * id resolves to nothing from now on (ids are never reused), so nothing is
     * rewritten here. */
    /* Every other holder lets go; the batch is then freed on return. */
    unlinkRetiredNodes(graphsInIdOrder, retiredBatch);
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
