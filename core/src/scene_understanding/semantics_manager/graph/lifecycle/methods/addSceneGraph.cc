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
 * @file            addSceneGraph.cc
 *
 * @brief           Implements SemanticsManager::addSceneGraph(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <shared_mutex>
#include <utility>

#include "map_state/map/map.h"
#include "scene_understanding/semantics_manager/graph/locking/private_functions/private_functions.h"
#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::addSceneGraph(map::Map &map_inout)
{
    unsigned long mapId{};
    if (map_inout.getId(mapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* The map owns its scene graph; only the registry entry is made here. */
    scene_graph::SceneGraph *p_sceneGraph = nullptr;
    if (map_inout.getSceneGraph(p_sceneGraph) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSceneGraph returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* The graph is initialised exclusive before it is registered: nobody can
     * hold it yet, so this never waits for a running cycle. */
    {
        std::map<unsigned long, scene_graph::SceneGraph *> newGraph{
            {mapId, p_sceneGraph}};
        GraphLockScope graphScope{};
        if (!acquireGraphLocks(newGraph,
                               GraphLockMode::GRAPH_LOCK_MODE_EXCLUSIVE,
                               graphScope))
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: an exclusive lock cannot fail to be granted.",
                         __func__);
        }

        /* A scene graph that held nodes may still be pointed at by other
         * nodes: its nodes are retired for the next sweep, not freed here. */
        retireSceneGraphContents(*p_sceneGraph, p_sceneGraph->retiredNodes);
        p_sceneGraph->nextAvailableFloorId = 0;
        p_sceneGraph->p_map                = &map_inout;

        /* One building per map until buildings are detected. */
        if (p_sceneGraph->buildings.find(0) == p_sceneGraph->buildings.end())
        {
            p_sceneGraph->buildings[0] =
                std::make_unique<scene_graph::Building>();
        }
    }
    std::unique_lock<std::shared_mutex> registryLock(registryMutex);
    sceneGraphs[mapId] = p_sceneGraph;
    erasedSceneGraphs.erase(std::remove(erasedSceneGraphs.begin(),
                                        erasedSceneGraphs.end(),
                                        p_sceneGraph),
                            erasedSceneGraphs.end());
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
