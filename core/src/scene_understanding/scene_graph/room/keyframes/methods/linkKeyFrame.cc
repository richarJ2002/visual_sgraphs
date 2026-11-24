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
 * @file            linkKeyFrame.cc
 *
 * @brief           Implements Room::linkKeyFrame(), declared in
 *                  scene_graph/room/objects/RoomClass.h.
 */

#include "scene_understanding/scene_graph/room/objects/RoomClass.h"

#include <algorithm>
#include <vector>

#include "map_state/keyframe/objects/KeyFrameClass.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::linkKeyFrame(keyframe::KeyFrame *p_keyFrame_inout,
                              Room               *p_previousRoom_inout,
                              Room               *p_nextRoom_inout)
{
    if (p_keyFrame_inout == nullptr)
    {
        return RoomStatus::ROOM_STATUS_INVALID_ARGUMENT;
    }

    if (p_previousRoom_inout != nullptr)
    {
        std::vector<keyframe::KeyFrame *> &previousKeyFrames =
            p_previousRoom_inout->keyFrames;
        previousKeyFrames.erase(std::remove(previousKeyFrames.begin(),
                                            previousKeyFrames.end(),
                                            p_keyFrame_inout),
                                previousKeyFrames.end());
    }

    p_keyFrame_inout->roomId = -1;
    if (p_nextRoom_inout != nullptr)
    {
        std::vector<keyframe::KeyFrame *> &nextKeyFrames =
            p_nextRoom_inout->keyFrames;
        if (std::find(nextKeyFrames.begin(),
                      nextKeyFrames.end(),
                      p_keyFrame_inout) == nextKeyFrames.end())
        {
            nextKeyFrames.push_back(p_keyFrame_inout);
        }
        p_keyFrame_inout->roomId = p_nextRoom_inout->id;
    }

    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
