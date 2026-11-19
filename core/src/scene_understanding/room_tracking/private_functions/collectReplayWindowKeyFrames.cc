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
 * @file            collectReplayWindowKeyFrames.cc
 *
 * @brief           Implements collectReplayWindowKeyFrames(), declared in
 *                  room_tracking/private_functions/private_functions.h.
 */

#include "scene_understanding/room_tracking/private_functions/private_functions.h"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <vector>

#include "map_state/keyframe/objects/KeyFrameClass.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"

namespace vs_graphs
{
namespace core
{

void collectReplayWindowKeyFrames(
    map::Map                          *p_map_in,
    std::vector<keyframe::KeyFrame *> &windowKeyFrames_out)
{
    windowKeyFrames_out.clear();
    if (p_map_in == nullptr)
    {
        return;
    }

    std::vector<keyframe::KeyFrame *> mapKeyFrames{};
    if (p_map_in->getAllKeyFrames(mapKeyFrames) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    /* Drop missing and bad key frames, keeping the map order. */
    windowKeyFrames_out.reserve(mapKeyFrames.size());
    for (keyframe::KeyFrame *p_keyFrame : mapKeyFrames)
    {
        bool keyFrameIsBad{};
        if (!(p_keyFrame == nullptr) &&
            p_keyFrame->isBad(keyFrameIsBad) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (!(p_keyFrame == nullptr || keyFrameIsBad))
        {
            windowKeyFrames_out.push_back(p_keyFrame);
        }
    }
    std::sort(windowKeyFrames_out.begin(),
              windowKeyFrames_out.end(),
              isKeyFrameEarlier);

    if (windowKeyFrames_out.size() > MAXIMUM_TRAVERSAL_HISTORY_KEYFRAMES)
    {
        windowKeyFrames_out.erase(windowKeyFrames_out.begin(),
                                  windowKeyFrames_out.end() -
                                      static_cast<std::ptrdiff_t>(
                                          MAXIMUM_TRAVERSAL_HISTORY_KEYFRAMES));
    }
}

} /* namespace core */
} /* namespace vs_graphs */
