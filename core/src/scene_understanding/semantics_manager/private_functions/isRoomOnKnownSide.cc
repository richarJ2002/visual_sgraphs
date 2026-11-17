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
 * @file            isRoomOnKnownSide.cc
 *
 * @brief           Implements isRoomOnKnownSide(), declared in
 *                  semantics_manager/private_functions/private_functions.h.
 */

#include "private_functions.h"

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus isRoomOnKnownSide(
    semantic::Room                               *p_room_in,
    const Eigen::Vector4d                        &passageEquation_world_in,
    const semantic::Passage::KnownSideProvenance &knownSide_in,
    double                                        knownSideSign_in,
    bool                                         &isOnKnownSide_out)
{
    isOnKnownSide_out = false;
    bool isKnownSideDirectional{};
    if (!(p_room_in == nullptr) &&
        knownSide_in.hasDirection(isKnownSideDirectional) !=
            semantic::KnownSideProvenanceStatus::
                KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_room_in == nullptr || !isKnownSideDirectional ||
        std::abs(knownSideSign_in) < 1e-8)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    Eigen::Vector3d roomCentroid{};
    if (p_room_in->getCentroid(roomCentroid) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const double roomSide_m =
        passageEquation_world_in.head<3>().dot(roomCentroid) +
        passageEquation_world_in(3);
    isOnKnownSide_out = roomSide_m * knownSideSign_in > 0.0;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
