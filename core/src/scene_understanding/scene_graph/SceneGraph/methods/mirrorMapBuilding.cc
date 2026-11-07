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
 * @file            mirrorMapBuilding.cc
 *
 * @brief           Implements SceneGraph::mirrorMapBuilding(), declared in
 *                  scene_graph/SceneGraph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphClass.h"

#include <map>
#include <rclcpp/logging.hpp>
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

SceneGraphStatus SceneGraph::mirrorMapBuilding(map::Map &map_inout,
                                               Building &building_out)
{
    std::vector<semantic::Floor *> mapFloors{};
    if (map_inout.getAllFloors(mapFloors) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    for (semantic::Floor *p_floor : mapFloors)
    {
        int                           floorId{};
        std::vector<semantic::Room *> floorRoomList{};
        if (p_floor == nullptr ||
            p_floor->getId(floorId) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS ||
            p_floor->getRooms(floorRoomList) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            continue;
        }
        building_out.floors.emplace(floorId, p_floor);

        /* A storey's passages are the passages its rooms list; a passage
         * whose two rooms sit on different storeys appears under both until
         * such passages are detected and held by the building. */
        std::map<int, semantic::Room *> &floorRoomsById =
            building_out.floorRooms[floorId];
        std::map<int, semantic::Passage *> &floorPassagesById =
            building_out.floorPassages[floorId];
        for (semantic::Room *p_room : floorRoomList)
        {
            int                              roomId{};
            std::vector<semantic::Passage *> roomPassages{};
            if (p_room == nullptr ||
                p_room->getId(roomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
                p_room->getPassages(roomPassages) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                continue;
            }
            floorRoomsById.emplace(roomId, p_room);

            for (semantic::Passage *p_passage : roomPassages)
            {
                int passageId{};
                if (p_passage != nullptr &&
                    p_passage->getId(passageId) ==
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    floorPassagesById.emplace(passageId, p_passage);
                }
            }
        }
    }

    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
