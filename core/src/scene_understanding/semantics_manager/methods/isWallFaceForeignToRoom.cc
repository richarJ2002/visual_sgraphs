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
 * @file            isWallFaceForeignToRoom.cc
 *
 * @brief           Implements SemanticsManager::isWallFaceForeignToRoom(),
 *                  declared in SemanticsManager.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantic/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/room/room.h"

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::isWallFaceForeignToRoom(semantic::Room   *p_room_in,
                                              geometric::Plane *p_wall_in,
                                              bool &isWallFaceForeignToRoom_out)
{
    bool room_inIsBad{};
    if (!(p_room_in == nullptr) &&
        p_room_in->isBad(room_inIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool wallIsBad{};
    if (!(p_room_in == nullptr || room_inIsBad || p_wall_in == nullptr) &&
        p_wall_in->isBad(wallIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_room_in == nullptr || room_inIsBad || p_wall_in == nullptr ||
        wallIsBad)
    {
        isWallFaceForeignToRoom_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    Eigen::Vector3d roomCentroid_world_m{};
    Eigen::Vector3d cameraCenter_world_m{};
    bool            hasCameraViewpoint{};
    if (getRoomViewpoints(p_room_in,
                          roomCentroid_world_m,
                          cameraCenter_world_m,
                          hasCameraViewpoint) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomViewpoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    bool isCentroidOnForeignSide{};
    if (isPointOnForeignSideOfWallFace(p_wall_in,
                                       roomCentroid_world_m,
                                       isCentroidOnForeignSide) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isPointOnForeignSideOfWallFace returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    bool isCameraOnForeignSide{};
    if (hasCameraViewpoint &&
        isPointOnForeignSideOfWallFace(p_wall_in,
                                       cameraCenter_world_m,
                                       isCameraOnForeignSide) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isPointOnForeignSideOfWallFace returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    /* For the room the camera is in, the camera and the centroid must agree. */
    isWallFaceForeignToRoom_out =
        isCentroidOnForeignSide &&
        (!hasCameraViewpoint || isCameraOnForeignSide);
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
