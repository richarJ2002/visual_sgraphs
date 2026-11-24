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
 * @file            updateRoomBoundaryStatus.cc
 *
 * @brief           Implements updateRoomBoundaryStatus(), declared in
 *                  room_validation/private_functions/private_functions.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/room_validation/private_functions/private_functions.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus updateRoomBoundaryStatus(
    semantic::Room                      *p_room_inout,
    const semantic::Room::BoundaryStatus boundaryStatus_in,
    const std::vector<Eigen::Vector3d>  &boundaryCorners_world_m_in)
{
    /* Refresh stored corners every cycle the loop is COMPLETE (even when
     * the status itself didn't change -- wall positions can still
     * drift), and clear them the moment it stops being COMPLETE. */
    if (p_room_inout->setBoundaryCorners_world_m(
            boundaryStatus_in == semantic::Room::BoundaryStatus::COMPLETE
                ? boundaryCorners_world_m_in
                : std::vector<Eigen::Vector3d>{}) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setBoundaryCorners_world_m returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    semantic::Room::BoundaryStatus previousBoundaryStatus{};
    if (p_room_inout->getBoundaryStatus(previousBoundaryStatus) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBoundaryStatus returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    if (previousBoundaryStatus == boundaryStatus_in)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    if (p_room_inout->setBoundaryStatus(boundaryStatus_in) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setBoundaryStatus returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Upper-case status name, as written in the boundary log line. */
    const char *boundaryStatusName = "unknown";
    switch (boundaryStatus_in)
    {
    case semantic::Room::BoundaryStatus::UNOBSERVED:
        boundaryStatusName = "UNOBSERVED";
        break;
    case semantic::Room::BoundaryStatus::INCOMPLETE:
        boundaryStatusName = "INCOMPLETE";
        break;
    case semantic::Room::BoundaryStatus::COMPLETE:
        boundaryStatusName = "COMPLETE";
        break;
    case semantic::Room::BoundaryStatus::CONFLICTING:
        boundaryStatusName = "CONFLICTING";
        break;
    }

    int roomId{};
    if (p_room_inout->getId(roomId) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<geometric::Plane *> roomWalls{};
    if (p_room_inout->getWalls(roomWalls) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::cout << "[SemMgr] Room#" << roomId
              << " boundary=" << boundaryStatusName << " (" << roomWalls.size()
              << " walls)" << std::endl;

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
