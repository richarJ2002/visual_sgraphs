/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            collectFloorObservation.cc
 *
 * @brief           Implements SemanticVerify::collectFloorObservation(),
 *                  declared in semantic_verify/objects/SemanticVerifyClass.h.
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include "scene_understanding/semantic/floor/floor.h"
#include "scene_understanding/semantic/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/semantic/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/room/room.h"

#include <optional>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticVerifyStatus SemanticVerify::collectFloorObservation(
    const Room                     *p_room_in,
    std::optional<Eigen::Vector4d> &floorPlane_world_out)
{
    floorPlane_world_out.reset();
    if (p_room_in == nullptr)
    {
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    Floor                              *p_floor = nullptr;
    std::optional<Floor::PlaneIdentity> floorIdentity{};
    Eigen::Vector3d                     roomCentroid_world_m{};
    if (p_room_in->getFloor(p_floor) != RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room_in->getCentroid(roomCentroid_world_m) !=
            RoomStatus::ROOM_STATUS_SUCCESS ||
        (p_floor != nullptr && p_floor->getPlaneIdentity(floorIdentity) !=
                                   FloorStatus::FLOOR_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: a room or floor getter returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!floorIdentity.has_value() ||
        !floorIdentity->planeEquation_world.allFinite() ||
        floorIdentity->planeEquation_world.head<3>().norm() < 1e-9)
    {
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    /* Same convention as the walls (Room::getWallNormalTowardRoom_world):
     * the room centroid evaluates positive, so the normal points up. */
    Eigen::Vector4d floorPlane_world =
        floorIdentity->planeEquation_world /
        floorIdentity->planeEquation_world.head<3>().norm();
    if (roomCentroid_world_m.allFinite() &&
        floorPlane_world.head<3>().dot(roomCentroid_world_m) +
                floorPlane_world[3] <
            0.0)
    {
        floorPlane_world = -floorPlane_world;
    }
    floorPlane_world_out = floorPlane_world;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
