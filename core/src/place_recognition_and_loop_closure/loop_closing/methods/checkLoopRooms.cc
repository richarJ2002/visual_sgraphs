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
 * @file            checkLoopRooms.cc
 *
 * @brief           Implements LoopClosing::checkLoopRooms(), declared in
 *                  loop_closing/objects/LoopClosingClass.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingClass.h"

#include <iostream>
#include <rclcpp/logging.hpp>

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

namespace vs_graphs
{
namespace core
{
namespace loop_closing
{

LoopClosingStatus LoopClosing::checkLoopRooms(bool &isLoopRejected_out)
{
    isLoopRejected_out                      = false;
    types::SystemParams *p_params           = nullptr;
    SemanticsManager    *p_semanticsManager = nullptr;
    map::Map            *p_loopMap          = nullptr;
    unsigned long        loopMapId{};
    if (types::SystemParams::getParams(p_params) !=
            types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS ||
        p_atlas->getSemanticsManager(p_semanticsManager) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS ||
        p_currentKF->getMap(p_loopMap) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS ||
        p_loopMap == nullptr ||
        p_loopMap->getId(loopMapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams/getSemanticsManager/getMap/getId returned "
                     "a failure status or no map; the loop passes.",
                     __func__);
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    /* Key frame rooms are read under the graph lock that guards them. A map
     * without a registered graph has no rooms: the loop passes. */
    GraphLockScope graphScope{};
    if (p_semanticsManager->lockGraphs({loopMapId},
                                       GraphLockMode::GRAPH_LOCK_MODE_SHARED,
                                       graphScope) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }
    const int currentRoomId = p_currentKF->roomId;
    const int loopRoomId    = p_loopMatchedKF->roomId;

    /* Unknown rooms (-1, or a room no longer live) always pass; one room
     * passes; two neighbouring rooms pass, as a key frame near a passage may
     * carry the room on its other side. */
    const char *p_decisionName = "PASS_UNKNOWN_ROOM";
    if (currentRoomId >= 0 && loopRoomId >= 0 && currentRoomId == loopRoomId)
    {
        p_decisionName = "PASS_SAME_ROOM";
    }
    else if (currentRoomId >= 0 && loopRoomId >= 0)
    {
        semantic::Room *p_currentRoom = nullptr;
        semantic::Room *p_loopRoom    = nullptr;
        bool            areNeighbours{};
        if (p_semanticsManager->getMapRoomById(loopMapId,
                                               currentRoomId,
                                               p_currentRoom) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
            p_semanticsManager->getMapRoomById(loopMapId,
                                               loopRoomId,
                                               p_loopRoom) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapRoomById found no scene graph for map "
                         "%lu; the loop passes.",
                         __func__,
                         loopMapId);
        }
        if (p_currentRoom != nullptr && p_loopRoom != nullptr &&
            p_currentRoom->getIsSeparatedFrom(p_loopRoom, areNeighbours) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getIsSeparatedFrom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentRoom != nullptr && p_loopRoom != nullptr)
        {
            p_decisionName =
                areNeighbours ? "PASS_NEIGHBOUR_ROOM" : "REJECT_ROOM_MISMATCH";
            isLoopRejected_out =
                !areNeighbours && p_params->mapMerge.isLoopRoomCheckEnabled;
        }
    }

    std::cout << "SG_PIPELINE {\"event\":\"loop_room_check\",\"map_id\":"
              << loopMapId << ",\"current_keyframe_id\":" << p_currentKF->id
              << ",\"loop_keyframe_id\":" << p_loopMatchedKF->id
              << ",\"current_room_id\":" << currentRoomId
              << ",\"loop_room_id\":" << loopRoomId << ",\"decision\":\""
              << p_decisionName << "\",\"enforced\":"
              << (p_params->mapMerge.isLoopRoomCheckEnabled ? "true" : "false")
              << ",\"rejected\":" << (isLoopRejected_out ? "true" : "false")
              << "}" << std::endl;

    if (isLoopRejected_out &&
        recordLoopCorrectionEvent(false, "room_mismatch") !=
            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: recordLoopCorrectionEvent returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
}

} /* namespace loop_closing */
} /* namespace core */
} /* namespace vs_graphs */
