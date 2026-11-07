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
 * @file            eraseRoom.cc
 *
 * @brief           Implements SceneGraph::eraseRoom(), declared in
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

SceneGraphStatus SceneGraph::eraseRoom(unsigned long   mapId_in,
                                       semantic::Room *p_room_in)
{
    std::unique_lock<std::shared_mutex> graphLock(graphMutex);
    std::map<unsigned long, std::unique_ptr<SceneMap>>::iterator
        sceneMapIterator = sceneMaps.find(mapId_in);
    if (sceneMapIterator == sceneMaps.end())
    {
        return SceneGraphStatus::SCENE_GRAPH_STATUS_MAP_NOT_FOUND;
    }

    /* Erased by pointer, as the map lists do, so a room whose id changed
     * after it was added still leaves the tree. */
    SceneMap &sceneMap = *sceneMapIterator->second;
    for (std::map<int, semantic::Room *>::iterator detectedIterator =
             sceneMap.detectedRooms.begin();
         detectedIterator != sceneMap.detectedRooms.end();)
    {
        detectedIterator = detectedIterator->second == p_room_in
                               ? sceneMap.detectedRooms.erase(detectedIterator)
                               : std::next(detectedIterator);
    }

    for (std::map<int, semantic::Room *>::iterator candidateIterator =
             sceneMap.candidateRooms.begin();
         candidateIterator != sceneMap.candidateRooms.end();)
    {
        candidateIterator =
            candidateIterator->second == p_room_in
                ? sceneMap.candidateRooms.erase(candidateIterator)
                : std::next(candidateIterator);
    }

    if ((sceneMap.p_map->eraseDetectedMapRoom(p_room_in) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (sceneMap.p_map->eraseMarkerBasedMapRoom(p_room_in) !=
         map::MapStatus::MAP_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: eraseDetectedMapRoom/eraseMarkerBasedMapRoom "
                     "returned a failure status although they "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
