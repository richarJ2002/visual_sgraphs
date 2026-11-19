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
 * @file            transferKeyFrames.cc
 *
 * @brief           Implements Room::transferKeyFrames(), declared in
 *                  scene_graph/room/objects/RoomClass.h.
 */

#include "scene_understanding/scene_graph/room/objects/RoomClass.h"

#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::transferKeyFrames(Room *p_survivorRoom_inout)
{
    if (p_survivorRoom_inout == this)
    {
        return RoomStatus::ROOM_STATUS_INVALID_ARGUMENT;
    }

    /* linkKeyFrame erases from this list, so walk a copy. */
    const std::vector<keyframe::KeyFrame *> movedKeyFrames = keyFrames;
    for (keyframe::KeyFrame *p_movedKeyFrame : movedKeyFrames)
    {
        if (linkKeyFrame(p_movedKeyFrame, this, p_survivorRoom_inout) !=
            RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: linkKeyFrame rejected a null key frame the "
                         "room list cannot hold; continuing as before.",
                         __func__);
        }
    }

    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
