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
 * @file            getFloorPassagesInIdOrder.cc
 *
 * @brief           Implements SemanticsManager::getFloorPassagesInIdOrder(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <shared_mutex>
#include <vector>

#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::getFloorPassagesInIdOrder(
    unsigned long                     mapId_in,
    int                               floorId_in,
    std::vector<semantic::Passage *> &passages_out) const
{
    passages_out.clear();
    std::shared_lock<std::shared_mutex> graphLock(registryMutex);
    std::map<unsigned long, scene_graph::SceneGraph *>::const_iterator
        sceneGraphIterator = sceneGraphs.find(mapId_in);
    if (sceneGraphIterator == sceneGraphs.end())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }
    semantic::Floor *p_floor =
        findFloorInBuildings(*sceneGraphIterator->second, floorId_in);

    std::vector<semantic::Room *> floorRoomList{};
    if (isFloorLive(p_floor) && p_floor->getRooms(floorRoomList) !=
                                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* A storey's passages are the passages its rooms list; the id map sorts
     * them and lists a passage shared by two rooms once. */
    std::map<int, semantic::Passage *> passagesById{};
    for (semantic::Room *p_room : floorRoomList)
    {
        std::vector<semantic::Passage *> roomPassages{};
        if (!isRoomLive(p_room) ||
            p_room->getPassages(roomPassages) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            continue;
        }
        for (semantic::Passage *p_passage : roomPassages)
        {
            int passageId{};
            if (isPassageLive(p_passage) &&
                p_passage->getId(passageId) ==
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                passagesById.emplace(passageId, p_passage);
            }
        }
    }
    for (const std::pair<const int, semantic::Passage *> &passageEntry :
         passagesById)
    {
        passages_out.push_back(passageEntry.second);
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
