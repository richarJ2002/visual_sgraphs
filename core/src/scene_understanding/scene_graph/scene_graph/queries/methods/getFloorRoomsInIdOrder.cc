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
 * @file            getFloorRoomsInIdOrder.cc
 *
 * @brief           Implements SceneGraph::getFloorRoomsInIdOrder(), declared in
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

SceneGraphStatus SceneGraph::getFloorRoomsInIdOrder(
    unsigned long                  mapId_in,
    int                            floorId_in,
    std::vector<semantic::Room *> &rooms_out) const
{
    rooms_out.clear();
    std::shared_lock<std::shared_mutex>                 graphLock(graphMutex);
    std::map<unsigned long, SceneMap *>::const_iterator sceneMapIterator =
        sceneMaps.find(mapId_in);
    if (sceneMapIterator == sceneMaps.end())
    {
        return SceneGraphStatus::SCENE_GRAPH_STATUS_MAP_NOT_FOUND;
    }
    semantic::Floor *p_floor =
        findFloorInBuildings(*sceneMapIterator->second, floorId_in);

    std::vector<semantic::Room *> floorRoomList{};
    if (isFloorLive(p_floor) && p_floor->getRooms(floorRoomList) !=
                                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* The storey's own list decides membership; the id map sorts it. */
    std::map<int, semantic::Room *> roomsById{};
    for (semantic::Room *p_room : floorRoomList)
    {
        int roomId{};
        if (isRoomLive(p_room) &&
            p_room->getId(roomId) == semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            roomsById.emplace(roomId, p_room);
        }
    }
    for (const std::pair<const int, semantic::Room *> &roomEntry : roomsById)
    {
        rooms_out.push_back(roomEntry.second);
    }
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
