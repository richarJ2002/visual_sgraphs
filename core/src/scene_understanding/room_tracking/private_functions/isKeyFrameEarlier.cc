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
 * @file            isKeyFrameEarlier.cc
 *
 * @brief           Implements isKeyFrameEarlier(), declared in
 *                  room_tracking/private_functions/private_functions.h.
 */

#include "map_state/keyframe/keyframe.h"

#include "scene_understanding/room_tracking/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

bool isKeyFrameEarlier(const keyframe::KeyFrame *p_firstKeyFrame_in,
                       const keyframe::KeyFrame *p_secondKeyFrame_in)
{
    if (p_firstKeyFrame_in->frameId != p_secondKeyFrame_in->frameId)
    {
        return p_firstKeyFrame_in->frameId < p_secondKeyFrame_in->frameId;
    }
    return p_firstKeyFrame_in->id < p_secondKeyFrame_in->id;
}

} /* namespace core */
} /* namespace vs_graphs */
