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
    /* The crossing results of this cycle's replay are consumed here. */
    keyframe::KeyFrame *p_crossingKeyFrame  = p_latestCrossingKeyFrame;
    const bool          isCrossingEntered   = isLatestCrossingEntered;
    const std::size_t   crossingLinkedCount = crossingLinkedKeyFrameCount;
    p_latestCrossingKeyFrame                = nullptr;
    isLatestCrossingEntered                 = false;
    crossingLinkedKeyFrameCount             = 0U;

    std::vector<keyframe::KeyFrame *> windowKeyFrames{};
    collectReplayWindowKeyFrames(p_activeMap_in, windowKeyFrames);
    if (windowKeyFrames.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    unsigned long activeMapId{};
    if (p_activeMap_in->getId(activeMapId) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* Key frames that left the window are never linked again, so their
     * crossing marks can go. */
    long unsigned int windowFirstKeyFrameId = windowKeyFrames.front()->id;
    for (keyframe::KeyFrame *p_windowKeyFrame : windowKeyFrames)
    {
        windowFirstKeyFrameId =
            std::min(windowFirstKeyFrameId, p_windowKeyFrame->id);
    }
    crossingLinkedKeyFrameIds.erase(
        crossingLinkedKeyFrameIds.begin(),
        crossingLinkedKeyFrameIds.lower_bound(windowFirstKeyFrameId));

    /* Why a key frame gets no room here: no current room yet (outdoor
     * start, before bootstrap, a fresh map after tracking loss), a current
     * room id the active map no longer holds, a current room flagged bad
     * (fused or retired this cycle), or a latest crossing that entered no
     * live room. */
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

    /* Key frames up to the latest crossing were taken before it, in a room
     * the current room does not tell; the replay links those after an
     * entering crossing. Only key frames after it are left to this linker. */
    const std::vector<keyframe::KeyFrame *>::iterator crossingIt =
        std::find(windowKeyFrames.begin(),
                  windowKeyFrames.end(),
                  p_crossingKeyFrame);
    const bool        hasCrossingInWindow = crossingIt != windowKeyFrames.end();
    const std::size_t fallbackStartIndex =
        hasCrossingInWindow
            ? static_cast<std::size_t>(crossingIt - windowKeyFrames.begin()) +
                  1U
            : 0U;
    const char *p_linkReason = "LINKED";
    if (hasCrossingInWindow && !isCrossingEntered)
    {
        p_linkReason = "CROSSING_ROOM_UNKNOWN";
    }
    else if (currentRoomIdSnapshot < 0)
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
    const bool canLinkToCurrentRoom =
        !(hasCrossingInWindow && !isCrossingEntered) &&
        p_currentRoom != nullptr && !isCurrentRoomBad;

    std::size_t       cycleLinkedCount        = 0U;
    std::size_t       unlinkedCount           = 0U;
    std::size_t       unlinkedBeforeCrossing  = 0U;
    long unsigned int unlinkedFirstKeyFrameId = 0U;
    long unsigned int unlinkedLastKeyFrameId  = 0U;
    for (std::size_t windowIndex = 0U; windowIndex < windowKeyFrames.size();
         windowIndex++)
    {
        keyframe::KeyFrame *p_windowKeyFrame = windowKeyFrames[windowIndex];
        const bool          isLinkable =
            windowIndex >= fallbackStartIndex && canLinkToCurrentRoom &&
            p_windowKeyFrame->roomId < 0 &&
            crossingLinkedKeyFrameIds.count(p_windowKeyFrame->id) == 0U;
        /* An unlinked key frame has no room to leave. */
        if (isLinkable && semantic::Room::linkKeyFrame(p_windowKeyFrame,
                                                       nullptr,
                                                       p_currentRoom) ==
                              semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            cycleLinkedCount++;
        }
        if (p_windowKeyFrame->roomId >= 0)
        {
            continue;
        }
        if (unlinkedCount == 0U)
        {
            unlinkedFirstKeyFrameId = p_windowKeyFrame->id;
        }
        unlinkedLastKeyFrameId = p_windowKeyFrame->id;
        unlinkedCount++;
        if (windowIndex < fallbackStartIndex)
        {
            unlinkedBeforeCrossing++;
        }
    }

    /* One line per cycle over the replay window. Unlinked key frames before
     * the latest crossing wait for a replay to cover them; the others share
     * the reason. */
    std::cout << "SG_PIPELINE {\"event\":\"keyframe_room_link\","
                 "\"semantic_cycle\":"
              << pipelineSemanticCycle << ",\"map_id\":" << activeMapId
              << ",\"window_first_keyframe_id\":" << windowKeyFrames.front()->id
              << ",\"window_last_keyframe_id\":" << windowKeyFrames.back()->id
              << ",\"window_keyframes\":" << windowKeyFrames.size()
              << ",\"room_id\":" << currentRoomIdSnapshot
              << ",\"crossing_linked\":" << crossingLinkedCount
              << ",\"cycle_linked\":" << cycleLinkedCount
              << ",\"unlinked\":" << unlinkedCount
              << ",\"unlinked_before_crossing\":" << unlinkedBeforeCrossing
              << ",\"unlinked_first_keyframe_id\":" << unlinkedFirstKeyFrameId
              << ",\"unlinked_last_keyframe_id\":" << unlinkedLastKeyFrameId
              << ",\"reason\":\"" << p_linkReason << "\"}" << std::endl;

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
