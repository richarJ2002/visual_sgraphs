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
 * @file            getSceneGraphSnapshot.cc
 *
 * @brief           Implements SemanticsManager::getSceneGraphSnapshot(),
 *                  declared in semantics_manager/objects/
 *                  SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <vector>

#include "scene_understanding/semantics_manager/graph/snapshots/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::getSceneGraphSnapshot(
    unsigned long       mapId_in,
    SceneGraphSnapshot &snapshot_out) const
{
    snapshot_out = SceneGraphSnapshot{};
    GraphLockScope               graphScope{};
    scene_graph::SceneGraph     *p_sceneGraph = nullptr;
    const SemanticsManagerStatus lockStatus =
        lockMapGraph(mapId_in,
                     GraphLockMode::GRAPH_LOCK_MODE_SHARED,
                     graphScope,
                     p_sceneGraph);
    if (lockStatus != SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        return lockStatus;
    }

    std::vector<semantic::Floor *>   floors{};
    std::vector<semantic::Room *>    rooms{};
    std::vector<semantic::Passage *> passages{};
    if (getMapFloorsInIdOrder(mapId_in, floors) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        getMapRoomsInIdOrder(mapId_in, rooms) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        getMapPassagesInIdOrder(mapId_in, passages) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: the node queries returned a failure status although "
                     "the graph is locked; continuing as before.",
                     __func__);
    }

    snapshot_out.mapId = mapId_in;
    snapshot_out.floors.resize(floors.size());
    for (std::size_t floorIndex = 0U; floorIndex < floors.size(); ++floorIndex)
    {
        captureFloorEntry(floors[floorIndex], snapshot_out.floors[floorIndex]);
    }
    snapshot_out.rooms.resize(rooms.size());
    for (std::size_t roomIndex = 0U; roomIndex < rooms.size(); ++roomIndex)
    {
        captureRoomEntry(rooms[roomIndex], snapshot_out.rooms[roomIndex]);
    }
    snapshot_out.passages.resize(passages.size());
    for (std::size_t passageIndex = 0U; passageIndex < passages.size();
         ++passageIndex)
    {
        capturePassageEntry(passages[passageIndex],
                            snapshot_out.passages[passageIndex]);
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
