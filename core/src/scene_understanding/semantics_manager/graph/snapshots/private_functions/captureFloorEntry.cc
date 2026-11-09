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
 * @file            captureFloorEntry.cc
 *
 * @brief           Implements captureFloorEntry(), declared in
 *                  semantics_manager/graph/snapshots/private_functions/
 *                  private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/snapshots/private_functions/private_functions.h"

#include <vector>

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void captureFloorEntry(const semantic::Floor *p_floor_in,
                       SceneGraphFloorEntry  &floorEntry_out)
{
    floorEntry_out = SceneGraphFloorEntry{};
    std::vector<semantic::Room *> rooms{};
    if (p_floor_in->getId(floorEntry_out.id) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS ||
        p_floor_in->getName(floorEntry_out.name) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS ||
        p_floor_in->getCentroid(floorEntry_out.centroid_world_m) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS ||
        p_floor_in->getPlaneIdentity(floorEntry_out.planeIdentity) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS ||
        p_floor_in->getRooms(rooms) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: a floor getter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    for (semantic::Room *p_room : rooms)
    {
        int roomId{};
        if (p_room != nullptr &&
            p_room->getId(roomId) == semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            floorEntry_out.roomIds.push_back(roomId);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
