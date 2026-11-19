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
 * @file            linkReplayedKeyFrame.cc
 *
 * @brief           Implements SemanticsManager::linkReplayedKeyFrame(),
 *                  declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <rclcpp/logging.hpp>

#include "map_state/keyframe/objects/KeyFrameClass.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::linkReplayedKeyFrame(keyframe::KeyFrame *p_keyFrame_inout,
                                           semantic::Room     *p_room_inout)
{
    /* The first crossing that covered a key frame decides its room; later
     * replays never move it. */
    if (crossingLinkedKeyFrameIds.count(p_keyFrame_inout->id) != 0U)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* A key frame the end-of-cycle linker gave the room current then leaves
     * that room. */
    semantic::Room *p_previousRoom = nullptr;
    if (p_keyFrame_inout->roomId >= 0 &&
        getRoomById(p_keyFrame_inout->roomId, p_previousRoom) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        p_previousRoom = nullptr;
    }
    if (semantic::Room::linkKeyFrame(p_keyFrame_inout,
                                     p_previousRoom,
                                     p_room_inout) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: linkKeyFrame rejected a replayed key frame that "
                     "cannot be null; continuing as before.",
                     __func__);
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    crossingLinkedKeyFrameIds.insert(p_keyFrame_inout->id);
    crossingLinkedKeyFrameCount++;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
