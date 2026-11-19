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
 * @file            linkNewKeyFramesToCurrentRoom.cc
 *
 * @brief           Implements
 *                  SemanticsManager::linkNewKeyFramesToCurrentRoom(),
 *                  declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <vector>

#include "map_state/keyframe/objects/KeyFrameClass.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

#include "scene_understanding/room_tracking/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::linkNewKeyFramesToCurrentRoom(map::Map *p_activeMap_in)
{
    if (p_activeMap_in == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::vector<keyframe::KeyFrame *> mapKeyFrames{};
    unsigned long                     activeMapId{};
    if (p_activeMap_in->getAllKeyFrames(mapKeyFrames) !=
            map::MapStatus::MAP_STATUS_SUCCESS ||
        p_activeMap_in->getId(activeMapId) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames/getId returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }

    /* Key frame ids grow with creation time across all maps, so the ids at
     * or above the cursor are the key frames taken since the last cycle. */
    std::vector<keyframe::KeyFrame *> newKeyFrames{};
    std::size_t                       badKeyFrameCount = 0U;
    long unsigned int nextCursorKeyFrameId = firstUnlinkedKeyFrameId;
    for (keyframe::KeyFrame *p_mapKeyFrame : mapKeyFrames)
    {
        if (p_mapKeyFrame == nullptr ||
            p_mapKeyFrame->id < firstUnlinkedKeyFrameId)
        {
            continue;
        }
        if (p_mapKeyFrame->id >= nextCursorKeyFrameId)
        {
            nextCursorKeyFrameId = p_mapKeyFrame->id + 1U;
        }
        bool isKeyFrameBad{};
        if (p_mapKeyFrame->isBad(isKeyFrameBad) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (isKeyFrameBad)
        {
            badKeyFrameCount++;
            continue;
        }
        newKeyFrames.push_back(p_mapKeyFrame);
    }
    firstUnlinkedKeyFrameId = nextCursorKeyFrameId;
    std::sort(newKeyFrames.begin(), newKeyFrames.end(), isKeyFrameEarlier);
    if (newKeyFrames.empty() && badKeyFrameCount == 0U)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Why the new key frames get no room, if they get none: no current
     * room yet (outdoor start, before bootstrap, a fresh map after tracking
     * loss), a current room id the active map no longer holds, or a current
     * room flagged bad (fused or retired this cycle). */
    int currentRoomIdSnapshot{};
    if (getCurrentRoomId(currentRoomIdSnapshot) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentRoomId returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    semantic::Room *p_currentRoom = nullptr;
    if (currentRoomIdSnapshot >= 0 &&
        getRoomById(currentRoomIdSnapshot, p_currentRoom) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        p_currentRoom = nullptr;
    }
    bool isCurrentRoomBad{};
    if (p_currentRoom != nullptr &&
        p_currentRoom->isBad(isCurrentRoomBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const char *p_linkReason = "LINKED";
    if (currentRoomIdSnapshot < 0)
    {
        p_linkReason = "NO_CURRENT_ROOM";
    }
    else if (p_currentRoom == nullptr)
    {
        p_linkReason = "ROOM_NOT_IN_MAP";
    }
    else if (isCurrentRoomBad)
    {
        p_linkReason = "ROOM_BAD";
    }

    std::size_t linkedKeyFrameCount = 0U;
    if (p_currentRoom != nullptr && !isCurrentRoomBad)
    {
        for (keyframe::KeyFrame *p_newKeyFrame : newKeyFrames)
        {
            /* A new key frame has no room yet, so there is no previous room
             * to leave. */
            if (p_newKeyFrame->roomId < 0 &&
                semantic::Room::linkKeyFrame(p_newKeyFrame,
                                             nullptr,
                                             p_currentRoom) ==
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                linkedKeyFrameCount++;
            }
        }
    }

    /* One line per cycle: the new key frames are the ids in
     * [first_keyframe_id, last_keyframe_id]; those not linked share the
     * reason. */
    std::cout << "SG_PIPELINE {\"event\":\"keyframe_room_link\","
                 "\"semantic_cycle\":"
              << pipelineSemanticCycle << ",\"map_id\":" << activeMapId
              << ",\"first_keyframe_id\":"
              << (newKeyFrames.empty() ? 0U : newKeyFrames.front()->id)
              << ",\"last_keyframe_id\":"
              << (newKeyFrames.empty() ? 0U : newKeyFrames.back()->id)
              << ",\"new_keyframes\":" << newKeyFrames.size()
              << ",\"bad_keyframes\":" << badKeyFrameCount
              << ",\"room_id\":" << currentRoomIdSnapshot
              << ",\"linked\":" << linkedKeyFrameCount
              << ",\"unlinked\":" << newKeyFrames.size() - linkedKeyFrameCount
              << ",\"reason\":\"" << p_linkReason << "\"}" << std::endl;

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
