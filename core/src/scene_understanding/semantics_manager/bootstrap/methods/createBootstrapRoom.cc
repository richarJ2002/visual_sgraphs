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
 * @file            createBootstrapRoom.cc
 *
 * @brief           Implements SemanticsManager::createBootstrapRoom(),
 *                  declared in
semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <utility>

#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/geo_sem_helpers/objects/GeoSemHelpersStatusEnum.h"

namespace vs_graphs
{
namespace core
{

semantic::Room *SemanticsManager::createBootstrapRoom(
    unsigned long          mapId_in,
    const Eigen::Vector3d &cameraPosition_world_m_in,
    const std::optional<semantic::RoomContextSnapshot> &recoveryContext_in)
{
    std::unique_ptr<semantic::Room> p_blankRoomCandidate{};
    if (utils::geo_sem_helpers::GeoSemHelpers::createBlankRoomCandidate(
            p_atlas,
            p_blankRoomCandidate,
            cameraPosition_world_m_in,
            recoveryContext_in.has_value()) !=
        utils::geo_sem_helpers::GeoSemHelpersStatus::
            GEO_SEM_HELPERS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: createBlankRoomCandidate returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    semantic::Room *p_room = p_blankRoomCandidate.get();
    if (p_room == nullptr)
    {
        return nullptr;
    }

    /* A recovered room is the room the robot was lost in, so it keeps that
     * room's id; the old map's final room and this map's starting room then
     * name the same room. A live room of this map holding the id keeps it,
     * and the recovered room keeps its fresh id. */
    semantic::Room *p_roomHoldingId = nullptr;
    if (recoveryContext_in.has_value() && recoveryContext_in->roomId >= 0 &&
        getMapRoomById(mapId_in, recoveryContext_in->roomId, p_roomHoldingId) ==
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS &&
        p_roomHoldingId == nullptr &&
        p_room->setId(recoveryContext_in->roomId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* The room is born confirmed: it enters the map as a candidate and is
     * promoted at once. */
    if (addCandidateRoom(mapId_in, std::move(p_blankRoomCandidate)) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        promoteCandidateRoom(mapId_in, p_room) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addCandidateRoom/promoteCandidateRoom returned a "
                     "failure status; continuing as before.",
                     __func__);
    }

    int roomId{};
    if (p_room->setRoomVariant(semantic::Room::RoomVariant::ROOM) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room->getId(roomId) != semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room->setName("Room#" + std::to_string(roomId)) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_room->setBoundaryStatus(semantic::Room::BoundaryStatus::UNOBSERVED) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: a room setter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (recoveryContext_in.has_value() &&
        p_room->setPreviouslyVisited(
            recoveryContext_in->wasPreviouslyVisited) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPreviouslyVisited returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    return p_room;
}

} /* namespace core */
} /* namespace vs_graphs */
