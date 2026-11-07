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
 * @file            resyncMap.cc
 *
 * @brief           Implements SceneGraph::resyncMap(), declared in
 *                  scene_graph/SceneGraph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphClass.h"

#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <shared_mutex>
#include <utility>
#include <vector>

#include "map_state/map/map.h"
#include "scene_understanding/scene_graph/Floor/Floor.h"
#include "scene_understanding/scene_graph/Passage/Passage.h"
#include "scene_understanding/scene_graph/Room/Room.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus SceneGraph::resyncMap(map::Map &map_inout)
{
    /* The map lists are read before the graph lock is taken, so the lock is
     * held only for the swap. */
    unsigned long                    mapId{};
    std::vector<semantic::Floor *>   mapFloors{};
    std::vector<semantic::Room *>    detectedRooms{};
    std::vector<semantic::Room *>    candidateRooms{};
    std::vector<semantic::Passage *> mapPassages{};
    if ((map_inout.getId(mapId) != map::MapStatus::MAP_STATUS_SUCCESS) ||
        (map_inout.getAllFloors(mapFloors) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (map_inout.getAllDetectedMapRooms(detectedRooms) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (map_inout.getAllCandidateMapRooms(candidateRooms) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (map_inout.getAllPassages(mapPassages) !=
         map::MapStatus::MAP_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: "
            "getId/getAllFloors/getAllDetectedMapRooms/getAllCandidateMapRooms/"
            "getAllPassages returned a failure status although they "
            "cannot fail; continuing as before.",
            __func__);
    }

    std::unique_ptr<SceneMap> p_sceneMap = std::make_unique<SceneMap>();
    p_sceneMap->p_map                    = &map_inout;

    /* One building per map until buildings are detected. */
    std::unique_ptr<Building> p_building = std::make_unique<Building>();
    for (semantic::Floor *p_floor : mapFloors)
    {
        int floorId{};
        if (p_floor != nullptr &&
            p_floor->getId(floorId) ==
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            p_building->floors.emplace(floorId, p_floor);
        }
    }
    p_sceneMap->buildings[0] = std::move(p_building);

    /* On an id held by two rooms of one list the first is kept and the
     * verification reports the clash as a count mismatch. */
    for (semantic::Room *p_room : detectedRooms)
    {
        int roomId{};
        if (p_room != nullptr &&
            p_room->getId(roomId) == semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            p_sceneMap->detectedRooms.emplace(roomId, p_room);
        }
    }
    for (semantic::Room *p_room : candidateRooms)
    {
        int roomId{};
        if (p_room != nullptr &&
            p_room->getId(roomId) == semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            p_sceneMap->candidateRooms.emplace(roomId, p_room);
        }
    }
    for (semantic::Passage *p_passage : mapPassages)
    {
        int passageId{};
        if (p_passage != nullptr &&
            p_passage->getId(passageId) ==
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            p_sceneMap->passages.emplace(passageId, p_passage);
        }
    }

    std::unique_lock<std::shared_mutex> graphLock(graphMutex);
    sceneMaps[mapId] = std::move(p_sceneMap);
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
