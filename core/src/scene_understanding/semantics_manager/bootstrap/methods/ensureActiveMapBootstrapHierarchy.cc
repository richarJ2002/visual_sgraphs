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
 * @file            ensureActiveMapBootstrapHierarchy.cc
 *
 * @brief           Implements
 *                  SemanticsManager::ensureActiveMapBootstrapHierarchy(),
 *                  declared in
semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <mutex>
#include <optional>
#include <rclcpp/logging.hpp>
#include <vector>

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/semantics_manager/bootstrap/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::ensureActiveMapBootstrapHierarchy(
    SemanticsManager::ActiveMapBootstrapResult &bootstrapResult_out,
    const std::optional<Eigen::Vector3d> &cameraPositionOverride_world_m_in)
{
    map::Map *p_atlasCurrentMap = nullptr;
    if ((p_atlas != nullptr) && p_atlas->getCurrentMap(p_atlasCurrentMap) !=
                                    atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    map::Map *p_activeMap = p_atlas != nullptr ? p_atlasCurrentMap : nullptr;
    if (p_activeMap == nullptr)
    {
        printBootstrapRecord("NO_ACTIVE_MAP",
                             std::nullopt,
                             std::nullopt,
                             std::nullopt,
                             std::nullopt,
                             pipelineSemanticCycle);
        bootstrapResult_out = ActiveMapBootstrapResult::NO_ACTIVE_MAP;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* The map's rooms, the current room and the room the robot was lost in
     * (set by a tracking-loss reset, -1 on a cold start). */
    std::vector<semantic::Room *> activeRooms{};
    unsigned long                 activeMapId{};
    int                           recoveryRoomId{};
    if (p_activeMap->getId(activeMapId) != map::MapStatus::MAP_STATUS_SUCCESS ||
        getMapRoomsInIdOrder(activeMapId, activeRooms) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        p_atlas->getCurrentSemanticRoomIdentity(recoveryRoomId) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: a map or room query returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    int currentRoomIdSnapshot = -1;
    {
        std::lock_guard<std::mutex> currentRoomLock(currentRoomMutex);
        currentRoomIdSnapshot = currentRoomId;
    }

    semantic::Room *p_bootstrapRoom =
        selectBootstrapRoom(activeRooms, currentRoomIdSnapshot, recoveryRoomId);
    const bool hadLiveCurrentRoom =
        findLiveRoomById(activeRooms, currentRoomIdSnapshot) != nullptr;

    /* A map without a room restores the last known room after a tracking
     * loss (its saved context), else starts one. */
    std::optional<semantic::RoomContextSnapshot> recoveryContext{};
    if (p_bootstrapRoom == nullptr && recoveryRoomId >= 0 &&
        p_atlas->copyLatestRoomContext(recoveryRoomId, recoveryContext) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: copyLatestRoomContext returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (recoveryContext.has_value())
    {
        return recoverRoomAfterTrackingLoss(bootstrapResult_out,
                                            p_activeMap,
                                            activeMapId,
                                            recoveryContext,
                                            hadLiveCurrentRoom,
                                            cameraPositionOverride_world_m_in);
    }

    bool initializedRoom = false;
    if (p_bootstrapRoom == nullptr)
    {
        /* An outdoor start has no room to begin with: the graph stays at the
         * environment node until a room is detected, which the next cycle
         * finds above as the first live room. */
        if (p_sysParams->semanticsManager.startEnvironment ==
            scene_graph::StartEnvironment::START_ENVIRONMENT_OUTDOOR)
        {
            printBootstrapRecord("AWAITING_FIRST_ROOM",
                                 activeMapId,
                                 std::nullopt,
                                 std::nullopt,
                                 std::nullopt,
                                 pipelineSemanticCycle);
            bootstrapResult_out = ActiveMapBootstrapResult::AWAITING_FIRST_ROOM;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }

        const std::optional<Eigen::Vector3d> cameraPosition_world_m =
            resolveBootstrapCameraPosition(p_activeMap,
                                           cameraPositionOverride_world_m_in,
                                           std::nullopt);
        if (!cameraPosition_world_m.has_value())
        {
            printBootstrapRecord("NO_USABLE_CAMERA_POSE",
                                 activeMapId,
                                 std::nullopt,
                                 std::nullopt,
                                 std::nullopt,
                                 pipelineSemanticCycle);
            bootstrapResult_out =
                ActiveMapBootstrapResult::NO_USABLE_CAMERA_POSE;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }

        p_bootstrapRoom = createBootstrapRoom(activeMapId,
                                              *cameraPosition_world_m,
                                              std::nullopt);
        if (p_bootstrapRoom == nullptr)
        {
            printBootstrapRecord("ROOM_CREATION_FAILED",
                                 activeMapId,
                                 std::nullopt,
                                 std::nullopt,
                                 std::nullopt,
                                 pipelineSemanticCycle);
            bootstrapResult_out =
                ActiveMapBootstrapResult::ROOM_CREATION_FAILED;
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }
        initializedRoom = true;
    }

    int bootstrapRoomId{};
    if (p_bootstrapRoom->getId(bootstrapRoomId) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    semantic::Floor *p_canonicalFloor =
        linkBootstrapRoomToFloor(p_activeMap,
                                 activeMapId,
                                 bootstrapRoomId,
                                 hadLiveCurrentRoom,
                                 p_bootstrapRoom);
    if (p_canonicalFloor == nullptr)
    {
        bootstrapResult_out = ActiveMapBootstrapResult::FLOOR_CREATION_FAILED;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    if (!initializedRoom)
    {
        bootstrapResult_out = ActiveMapBootstrapResult::REUSED;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    int canonicalFloorId{};
    if (p_canonicalFloor->getId(canonicalFloorId) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    printBootstrapRecord("BOOTSTRAP_CREATED",
                         activeMapId,
                         bootstrapRoomId,
                         canonicalFloorId,
                         std::nullopt,
                         pipelineSemanticCycle);
    bootstrapResult_out = ActiveMapBootstrapResult::INITIALIZED;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
