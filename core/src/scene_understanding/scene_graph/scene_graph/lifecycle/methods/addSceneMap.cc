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
 * @file            addSceneMap.cc
 *
 * @brief           Implements SceneGraph::addSceneMap(), declared in
 *                  scene_graph/scene_graph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/scene_graph/objects/SceneGraphClass.h"

#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <shared_mutex>
#include <utility>

#include "map_state/map/map.h"
#include "scene_understanding/scene_graph/scene_graph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus SceneGraph::addSceneMap(map::Map &map_inout)
{
    unsigned long mapId{};
    if (map_inout.getId(mapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* The map owns its scene map; only the registry entry is made here. */
    SceneMap *p_sceneMap = nullptr;
    if (map_inout.getSceneMap(p_sceneMap) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSceneMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    std::unique_lock<std::shared_mutex> graphLock(graphMutex);

    /* A scene map that held nodes may still be pointed at by other nodes:
     * its nodes are retired for the next sweep, not freed here. */
    retireSceneMapContents(*p_sceneMap, retiredNodes);
    p_sceneMap->nextAvailableFloorId = 0;
    p_sceneMap->p_map                = &map_inout;

    /* One building per map until buildings are detected. */
    if (p_sceneMap->buildings.find(0) == p_sceneMap->buildings.end())
    {
        p_sceneMap->buildings[0] = std::make_unique<Building>();
    }
    sceneMaps[mapId] = p_sceneMap;
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
