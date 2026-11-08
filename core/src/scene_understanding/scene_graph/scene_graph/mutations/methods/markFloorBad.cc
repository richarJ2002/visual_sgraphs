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
 * @file            markFloorBad.cc
 *
 * @brief           Implements SceneGraph::markFloorBad(), declared in
 *                  scene_graph/scene_graph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/scene_graph/objects/SceneGraphClass.h"

#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <shared_mutex>
#include <vector>

#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/scene_graph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus SceneGraph::markFloorBad(unsigned long    mapId_in,
                                          semantic::Floor *p_floor_inout)
{
    if (p_floor_inout == nullptr)
    {
        return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
    }
    std::unique_lock<std::shared_mutex> graphLock(graphMutex);

    std::vector<semantic::Room *> floorRooms{};
    if (p_floor_inout->getRooms(floorRooms) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS ||
        p_floor_inout->setBad() != semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRooms/setBad returned a failure status although "
                     "they cannot fail; continuing as before.",
                     __func__);
    }

    const std::map<unsigned long, SceneMap *>::const_iterator sceneMapIterator =
        sceneMaps.find(mapId_in);
    if (sceneMapIterator == sceneMaps.end())
    {
        return SceneGraphStatus::SCENE_GRAPH_STATUS_MAP_NOT_FOUND;
    }

    /* A bad parent means bad children: every room of the storey goes with it
     * and takes its dependent passages along. */
    for (semantic::Room *p_room : floorRooms)
    {
        flagRoomBad(*sceneMapIterator->second, p_room);
    }
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
