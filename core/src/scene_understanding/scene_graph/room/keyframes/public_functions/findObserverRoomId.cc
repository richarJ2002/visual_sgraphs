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

#include <map>
#include <rclcpp/logging.hpp>

#include "map_state/keyframe/objects/KeyFrameClass.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus findObserverRoomId(geometric::Plane *p_plane_in,
                              int              &observerRoomId_out)
{
    observerRoomId_out = -1;
    if (p_plane_in == nullptr)
    {
        return RoomStatus::ROOM_STATUS_SUCCESS;
    }

    std::map<keyframe::KeyFrame *, geometric::Plane::Observation>
        planeObservations{};
    if (p_plane_in->getObservations(planeObservations) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservations returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Votes and latest observer id per room, in room id order. */
    std::map<int, std::size_t>       roomVoteCounts{};
    std::map<int, long unsigned int> roomLatestObserverIds{};
    for (const std::pair<keyframe::KeyFrame *const,
                         geometric::Plane::Observation> &planeObservation :
         planeObservations)
    {
        keyframe::KeyFrame *p_observer = planeObservation.first;
        bool                isObserverBad{};
        if (p_observer != nullptr &&
            p_observer->isBad(isObserverBad) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_observer == nullptr || isObserverBad || p_observer->roomId < 0)
        {
            continue;
        }
        roomVoteCounts[p_observer->roomId]++;
        long unsigned int &latestObserverId =
            roomLatestObserverIds[p_observer->roomId];
        if (p_observer->id > latestObserverId)
        {
            latestObserverId = p_observer->id;
        }
    }

    std::size_t       bestVoteCount        = 0U;
    long unsigned int bestLatestObserverId = 0U;
    for (const std::pair<const int, std::size_t> &roomVote : roomVoteCounts)
    {
        const long unsigned int latestObserverId =
            roomLatestObserverIds[roomVote.first];
        if (roomVote.second > bestVoteCount ||
            (roomVote.second == bestVoteCount &&
             latestObserverId > bestLatestObserverId))
        {
            bestVoteCount        = roomVote.second;
            bestLatestObserverId = latestObserverId;
            observerRoomId_out   = roomVote.first;
        }
    }

    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
