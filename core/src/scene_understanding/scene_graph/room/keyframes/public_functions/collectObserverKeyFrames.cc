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
 * @file            collectObserverKeyFrames.cc
 *
 * @brief           Implements collectObserverKeyFrames(), declared in
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

RoomStatus collectObserverKeyFrames(
    geometric::Plane                  *p_plane_in,
    std::vector<keyframe::KeyFrame *> &observerKeyFrames_out)
{
    observerKeyFrames_out.clear();
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
    /* Keyed by id, so the observers come out in id order. */
    std::map<long unsigned int, keyframe::KeyFrame *> observersById{};
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
        if (p_observer != nullptr && !isObserverBad && p_observer->roomId >= 0)
        {
            observersById[p_observer->id] = p_observer;
        }
    }
    for (const std::pair<const long unsigned int, keyframe::KeyFrame *>
             &observerEntry : observersById)
    {
        observerKeyFrames_out.push_back(observerEntry.second);
    }
    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
