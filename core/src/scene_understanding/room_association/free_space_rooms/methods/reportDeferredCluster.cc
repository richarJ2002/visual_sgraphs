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
 * @file            reportDeferredCluster.cc
 *
 * @brief           Implements SemanticsManager::reportDeferredCluster(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/objects/SemanticsManagerClass.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::reportDeferredCluster(std::size_t clusterId_in)
{
    /*!
     * @brief           Axiom: every room after the first must be
     *                  discovered through a passage (a prospective-room
     *                  handle, checked above via
     *                  p_clusterProspective/matched wall ownership),
     *                  not conjured directly from free-space geometry
     *                  alone -- "if a wall is observed it must be
     *                  linked to a room; if that room is new, it must
     *                  be observed through a passage" (user rule). The
     *                  exception is the first room -- but per-MAP, not
     *                  per-mission: every tracking-loss reset starts an
     *                  entirely new Map with no passages yet either, so
     *                  it needs its own bootstrap room the same way the
     *                  mission's very first map did.
     *                  Atlas::GetAllRooms() spans every map (confirmed
     *                  by reading it), so scoping this to the CURRENT
     *                  map only is required -- otherwise a confirmed
     *                  room surviving in an old, now-inactive map would
     *                  permanently block every future map from ever
     *                  bootstrapping its own first room.
     */
    map::Map *p_currentMapForBootstrapCheck = nullptr;
    if (p_atlas->getCurrentMap(p_currentMapForBootstrapCheck) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> currentMapForBootstrapCheckAllRooms{};
    unsigned long                 bootstrapCheckMapId{};
    if ((p_currentMapForBootstrapCheck != nullptr) &&
        p_currentMapForBootstrapCheck->getId(bootstrapCheckMapId) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if ((p_currentMapForBootstrapCheck != nullptr) &&
        getMapRoomsInIdOrder(bootstrapCheckMapId,
                             currentMapForBootstrapCheckAllRooms) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapRoomsInIdOrder returned a failure "
                     "status; continuing as before.",
                     __func__);
    }
    const std::vector<semantic::Room *> currentMapRooms =
        p_currentMapForBootstrapCheck != nullptr
            ? currentMapForBootstrapCheckAllRooms
            : std::vector<semantic::Room *>();
    bool anyConfirmedRoomExistsInCurrentMap = false;
    for (vs_graphs::core::semantic::Room *p_existingRoom : currentMapRooms)
    {
        bool existingRoomIsBad{};
        if ((p_existingRoom != nullptr) &&
            p_existingRoom->isBad(existingRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room::RoomVariant existingRoomRoomVariant{};
        if ((p_existingRoom != nullptr && !existingRoomIsBad) &&
            p_existingRoom->getRoomVariant(existingRoomRoomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_existingRoom != nullptr && !existingRoomIsBad &&
            existingRoomRoomVariant ==
                vs_graphs::core::semantic::Room::RoomVariant::ROOM)
        {
            anyConfirmedRoomExistsInCurrentMap = true;
            break;
        }
    }

    if (anyConfirmedRoomExistsInCurrentMap)
    {
        std::cout << "[SemMgr] Cluster " << clusterId_in
                  << " matches no existing room, wall owner, or "
                     "passage-linked prospective room; deferring "
                     "(not the first room of this map, so it must be "
                     "discovered through a passage, not created from "
                     "geometry alone)."
                  << std::endl;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Reset transient: the active map was just cleared (no live ROOM)
     * but a last-known hierarchy exists. Bootstrap owns first-room
     * creation from that hierarchy; free-space must not conjure a
     * fresh SE# from stale cross-map walls/clusters in this cycle. */
    int atlasGetCurrentSemanticRoomIdentity{};
    if ((p_atlas != nullptr) && p_atlas->getCurrentSemanticRoomIdentity(
                                    atlasGetCurrentSemanticRoomIdentity) !=
                                    atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentSemanticRoomIdentity returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    const int pendingRecoveryRoomId =
        p_atlas != nullptr ? atlasGetCurrentSemanticRoomIdentity : -1;
    std::optional<semantic::RoomContextSnapshot> atlasRoomContext{};
    if ((pendingRecoveryRoomId >= 0) &&
        p_atlas->copyLatestRoomContext(pendingRecoveryRoomId,
                                       atlasRoomContext) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: copyLatestRoomContext returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (pendingRecoveryRoomId >= 0 && atlasRoomContext.has_value())
    {
        std::cout << "[SemMgr] Cluster " << clusterId_in
                  << " deferred: recovery Room#" << pendingRecoveryRoomId
                  << " owns first-room creation on this map." << std::endl;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /*!
     * @brief           First-room creation lives in the bootstrap
     *                  hierarchy, which runs before free-space
     *                  detection every cycle and promotes its room
     *                  immediately: a cluster that matches no room,
     *                  wall owner, or passage-linked prospective at
     *                  this point describes no known space, so it is
     *                  deferred rather than conjured into a duplicate
     *                  first room.
     */
    std::cout << "[SemMgr] Cluster " << clusterId_in
              << " deferred: first-room creation is owned by "
                 "bootstrap; cluster matches no known space."
              << std::endl;

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
