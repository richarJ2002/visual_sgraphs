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

/*!
 * @file            getMapRoomSnapshots.cc
 *
 * @brief           Implements SemanticsManager::getMapRoomSnapshots(), declared
 *                  in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <set>
#include <vector>

#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/semantics_manager/objects/RoomSnapshotStruct.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::getMapRoomSnapshots(
    unsigned long              mapId_in,
    GraphLockMode              lockMode_in,
    std::vector<RoomSnapshot> &roomSnapshots_out) const
{
    roomSnapshots_out.clear();
    GraphLockScope               graphScope{};
    scene_graph::SceneGraph     *p_sceneGraph = nullptr;
    const SemanticsManagerStatus lockStatus =
        lockMapGraph(mapId_in, lockMode_in, graphScope, p_sceneGraph);
    if (lockStatus != SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        return lockStatus;
    }

    std::vector<semantic::Room *> detectedRooms{};
    std::vector<semantic::Room *> allRooms{};
    if (getMapDetectedRoomsInIdOrder(mapId_in, detectedRooms) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        getMapRoomsInIdOrder(mapId_in, allRooms) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: the room queries returned a failure status although "
                     "the graph is locked; continuing as before.",
                     __func__);
    }
    const std::set<semantic::Room *> detectedRoomSet(detectedRooms.begin(),
                                                     detectedRooms.end());

    for (semantic::Room *p_room : allRooms)
    {
        RoomSnapshot                    roomSnapshot{};
        std::vector<geometric::Plane *> walls{};
        if (p_room->getId(roomSnapshot.id) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            p_room->isBoundaryComplete(roomSnapshot.isBoundaryComplete) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            p_room->getCentroid(roomSnapshot.centroid_world_m) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            p_room->getWalls(walls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: a room getter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_wall : walls)
        {
            int wallId{};
            if (p_wall->getId(wallId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            roomSnapshot.wallPlaneIds.push_back(wallId);
        }
        roomSnapshot.isDetected = detectedRoomSet.count(p_room) != 0U;
        roomSnapshots_out.push_back(roomSnapshot);
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
