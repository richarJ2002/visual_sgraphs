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
 * @file            eraseFloor.cc
 *
 * @brief           Implements SceneGraph::eraseFloor(), declared in
 *                  scene_graph/SceneGraph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphClass.h"

#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <shared_mutex>

#include "map_state/map/map.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus SceneGraph::eraseFloor(unsigned long    mapId_in,
                                        semantic::Floor *p_floor_in)
{
    std::unique_lock<std::shared_mutex> graphLock(graphMutex);
    std::map<unsigned long, std::unique_ptr<SceneMap>>::iterator
        sceneMapIterator = sceneMaps.find(mapId_in);
    if (sceneMapIterator == sceneMaps.end())
    {
        return SceneGraphStatus::SCENE_GRAPH_STATUS_MAP_NOT_FOUND;
    }

    SceneMap &sceneMap = *sceneMapIterator->second;
    for (std::pair<const int, std::unique_ptr<Building>> &buildingEntry :
         sceneMap.buildings)
    {
        for (std::map<int, semantic::Floor *>::iterator floorIterator =
                 buildingEntry.second->floors.begin();
             floorIterator != buildingEntry.second->floors.end();)
        {
            floorIterator =
                floorIterator->second == p_floor_in
                    ? buildingEntry.second->floors.erase(floorIterator)
                    : std::next(floorIterator);
        }
    }
    if (sceneMap.p_map->eraseMapFloor(p_floor_in) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: eraseMapFloor returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
