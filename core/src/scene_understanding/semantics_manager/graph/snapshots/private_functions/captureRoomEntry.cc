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
 * @file            captureRoomEntry.cc
 *
 * @brief           Implements captureRoomEntry(), declared in
 *                  semantics_manager/graph/snapshots/private_functions/
 *                  private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/snapshots/private_functions/private_functions.h"

#include <vector>

#include <rclcpp/logging.hpp>

#include "mapping/geometric/Plane/Plane.h"

namespace vs_graphs
{
namespace core
{

void captureRoomEntry(semantic::Room      *p_room_inout,
                      SceneGraphRoomEntry &roomEntry_out)
{
    roomEntry_out = SceneGraphRoomEntry{};
    std::vector<geometric::Plane *>  walls{};
    std::vector<semantic::Passage *> passages{};
    semantic::Floor                 *p_floor = nullptr;
    if (p_room_inout->getId(roomEntry_out.id) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_inout->getName(roomEntry_out.name) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_inout->isBad(roomEntry_out.isBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_inout->getRoomVariant(roomEntry_out.variant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_inout->getBoundaryStatus(roomEntry_out.boundaryStatus) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_inout->isBoundaryComplete(roomEntry_out.isBoundaryComplete) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_inout->hasPreviouslyVisited(roomEntry_out.hasBeenVisited) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_inout->getCentroid(roomEntry_out.centroid_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_inout->getBoundaryCorners_world_m(
            roomEntry_out.boundaryCorners_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_inout->getWalls(walls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_inout->getPassages(passages) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_inout->getFloor(p_floor) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: a room getter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    for (geometric::Plane *p_wall : walls)
    {
        int wallId{};
        if (p_wall != nullptr &&
            p_wall->getId(wallId) ==
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            roomEntry_out.wallPlaneIds.push_back(wallId);
        }
    }
    for (semantic::Passage *p_passage : passages)
    {
        int passageId{};
        if (p_passage != nullptr &&
            p_passage->getId(passageId) ==
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            roomEntry_out.passageIds.push_back(passageId);
        }
    }
    int floorId{};
    if (p_floor != nullptr &&
        p_floor->getId(floorId) == semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        roomEntry_out.floorId = floorId;
    }
}

} /* namespace core */
} /* namespace vs_graphs */
