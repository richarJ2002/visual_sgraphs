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
 * @file            mirrorMapIndexes.cc
 *
 * @brief           Implements SceneGraph::mirrorMapIndexes(), declared in
 *                  scene_graph/SceneGraph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphClass.h"

#include <rclcpp/logging.hpp>
#include <vector>

#include "map_state/map/map.h"
#include "scene_understanding/scene_graph/Passage/Passage.h"
#include "scene_understanding/scene_graph/Room/Room.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus SceneGraph::mirrorMapIndexes(map::Map &map_inout,
                                              SceneMap &sceneMap_inout)
{
    std::vector<semantic::Room *>    mapRooms{};
    std::vector<semantic::Room *>    candidateRooms{};
    std::vector<semantic::Passage *> mapPassages{};
    if ((map_inout.getAllDetectedMapRooms(mapRooms) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (map_inout.getAllCandidateMapRooms(candidateRooms) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (map_inout.getAllPassages(mapPassages) !=
         map::MapStatus::MAP_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllDetectedMapRooms/getAllCandidateMapRooms/"
                     "getAllPassages returned a failure status although they "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Detected rooms first: on an id clash the detected room is kept and the
     * verification reports the clash as a count mismatch. */
    mapRooms.insert(mapRooms.end(),
                    candidateRooms.begin(),
                    candidateRooms.end());
    for (semantic::Room *p_room : mapRooms)
    {
        int roomId{};
        if (p_room != nullptr &&
            p_room->getId(roomId) == semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            sceneMap_inout.rooms.emplace(roomId, p_room);
        }
    }

    for (semantic::Passage *p_passage : mapPassages)
    {
        int passageId{};
        if (p_passage != nullptr &&
            p_passage->getId(passageId) ==
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            sceneMap_inout.passages.emplace(passageId, p_passage);
        }
    }

    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
