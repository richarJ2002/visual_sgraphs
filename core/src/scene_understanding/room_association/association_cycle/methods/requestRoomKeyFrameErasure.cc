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
 * @file            requestRoomKeyFrameErasure.cc
 *
 * @brief           Implements SemanticsManager::requestRoomKeyFrameErasure(),
 *                  declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <mutex>
#include <rclcpp/logging.hpp>
#include <vector>

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::requestRoomKeyFrameErasure(map::Map       *p_map_in,
                                                 semantic::Room *p_room_in)
{
    if (!p_sysParams->semantics.isDeletedRoomKeyFrameErasureEnabled)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    int                               roomId{};
    std::vector<keyframe::KeyFrame *> mapKeyFrames{};
    if (p_room_in->getId(roomId) != semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
        p_map_in->getAllKeyFrames(mapKeyFrames) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId/getAllKeyFrames returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }

    /* KeyFrame::roomId is read under the cycle's exclusive graph lock. */
    std::vector<unsigned long> roomKeyFrameIds{};
    for (keyframe::KeyFrame *p_keyFrame : mapKeyFrames)
    {
        bool keyFrameIsBad{true};
        if (p_keyFrame != nullptr &&
            p_keyFrame->isBad(keyFrameIsBad) ==
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS &&
            !keyFrameIsBad && p_keyFrame->roomId == roomId)
        {
            roomKeyFrameIds.push_back(p_keyFrame->id);
        }
    }
    if (roomKeyFrameIds.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    {
        std::unique_lock<std::mutex> lock(keyFrameErasureMutex);
        keyFrameErasureRequests.insert(keyFrameErasureRequests.end(),
                                       roomKeyFrameIds.begin(),
                                       roomKeyFrameIds.end());
    }
    RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                "[SemMgr] Room#%d deleted: %zu key frames requested for "
                "erasure.",
                roomId,
                roomKeyFrameIds.size());
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
