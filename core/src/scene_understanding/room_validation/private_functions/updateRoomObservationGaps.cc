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
 * @file            updateRoomObservationGaps.cc
 *
 * @brief           Implements updateRoomObservationGaps(), declared in
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

SemanticsManagerStatus updateRoomObservationGaps(
    semantic::Room                         *p_room_inout,
    const std::vector<FiniteWallSegment2d> &wallSegments_in,
    const Eigen::Vector3d                  &groundAxisU_world_in,
    const Eigen::Vector3d                  &groundAxisV_world_in)
{
    Eigen::Vector3d gapCentroid_world_m{};
    if (p_room_inout->getCentroid(gapCentroid_world_m) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (gapCentroid_world_m.allFinite())
    {
        const Eigen::Vector2d gapCentroidGround_m(
            gapCentroid_world_m.dot(groundAxisU_world_in),
            gapCentroid_world_m.dot(groundAxisV_world_in));
        std::vector<semantic::Room::ObservationGap> roomObservationGaps{};
        if (computeRoomObservationGaps(wallSegments_in,
                                       gapCentroidGround_m,
                                       roomObservationGaps) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: computeRoomObservationGaps returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }
        if (p_room_inout->setObservationGaps(roomObservationGaps) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setObservationGaps returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    else
    {
        if (p_room_inout->setObservationGaps({}) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setObservationGaps returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
