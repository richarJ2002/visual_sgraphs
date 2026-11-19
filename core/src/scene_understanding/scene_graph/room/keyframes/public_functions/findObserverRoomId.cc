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
 * @file            findObserverRoomId.cc
 *
 * @brief           Implements findObserverRoomId(), declared in
 *                  scene_graph/room/keyframes/public_functions/
 *                  public_functions.h.
 */

#include "scene_understanding/scene_graph/room/keyframes/public_functions/public_functions.h"

#include <rclcpp/logging.hpp>
#include <utility>
#include <vector>

#include "map_state/keyframe/objects/KeyFrameClass.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus findObserverRoomId(geometric::Plane *p_plane_in,
                              int              &observerRoomId_out)
{
    std::vector<keyframe::KeyFrame *> observerKeyFrames{};
    if (collectObserverKeyFrames(p_plane_in, observerKeyFrames) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: collectObserverKeyFrames returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Each observer votes for the room it was taken in. */
    std::vector<std::pair<int, long unsigned int>> roomVotes{};
    for (keyframe::KeyFrame *p_observer : observerKeyFrames)
    {
        roomVotes.emplace_back(p_observer->roomId, p_observer->id);
    }
    return selectMajorityRoomId(roomVotes, observerRoomId_out);
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
