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
 * @file            recoverRoomAfterTrackingLoss.cc
 *
 * @brief           Implements SemanticsManager::recoverRoomAfterTrackingLoss(),
 *                  declared in
 * semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <optional>
#include <rclcpp/logging.hpp>

#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/semantics_manager/bootstrap/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::recoverRoomAfterTrackingLoss(
    SemanticsManager::ActiveMapBootstrapResult         &bootstrapResult_out,
    map::Map                                           *p_activeMap_inout,
    unsigned long                                       activeMapId_in,
    const std::optional<semantic::RoomContextSnapshot> &recoveryContext_in,
    bool                                                isCurrentRoomLive_in,
    const std::optional<Eigen::Vector3d> &cameraPositionOverride_world_m_in)
{
    const std::optional<Eigen::Vector3d> cameraPosition_world_m =
        resolveBootstrapCameraPosition(p_activeMap_inout,
                                       cameraPositionOverride_world_m_in,
                                       recoveryContext_in);
    if (!cameraPosition_world_m.has_value())
    {
        printBootstrapRecord("NO_USABLE_CAMERA_POSE",
                             activeMapId_in,
                             std::nullopt,
                             std::nullopt,
                             std::nullopt,
                             pipelineSemanticCycle);
        bootstrapResult_out = ActiveMapBootstrapResult::NO_USABLE_CAMERA_POSE;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    semantic::Room *p_recoveredRoom =
        createBootstrapRoom(activeMapId_in,
                            *cameraPosition_world_m,
                            recoveryContext_in);
    if (p_recoveredRoom == nullptr)
    {
        printBootstrapRecord("ROOM_CREATION_FAILED",
                             activeMapId_in,
                             std::nullopt,
                             std::nullopt,
                             std::nullopt,
                             pipelineSemanticCycle);
        bootstrapResult_out = ActiveMapBootstrapResult::ROOM_CREATION_FAILED;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    int recoveredRoomId{};
    if (p_recoveredRoom->getId(recoveredRoomId) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    semantic::Floor *p_canonicalFloor =
        linkBootstrapRoomToFloor(p_activeMap_inout,
                                 activeMapId_in,
                                 recoveredRoomId,
                                 isCurrentRoomLive_in,
                                 p_recoveredRoom);
    if (p_canonicalFloor == nullptr)
    {
        bootstrapResult_out = ActiveMapBootstrapResult::FLOOR_CREATION_FAILED;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const std::size_t restoredPassageCount =
        restoreRecoveryPassages(p_activeMap_inout,
                                activeMapId_in,
                                *recoveryContext_in,
                                p_recoveredRoom);

    int canonicalFloorId{};
    if (p_canonicalFloor->getId(canonicalFloorId) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    printBootstrapRecord("RECOVERY_RESTORED",
                         activeMapId_in,
                         recoveredRoomId,
                         canonicalFloorId,
                         restoredPassageCount,
                         pipelineSemanticCycle);
    bootstrapResult_out = ActiveMapBootstrapResult::RECOVERED;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
