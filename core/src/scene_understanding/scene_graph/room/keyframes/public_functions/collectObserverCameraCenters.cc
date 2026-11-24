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
 * @file            collectObserverCameraCenters.cc
 *
 * @brief           Implements collectObserverCameraCenters(), declared in
 *                  scene_graph/room/keyframes/public_functions/
 *                  public_functions.h.
 */

#include "scene_understanding/scene_graph/room/keyframes/public_functions/public_functions.h"

#include <rclcpp/logging.hpp>

#include "map_state/keyframe/objects/KeyFrameClass.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus collectObserverCameraCenters(
    geometric::Plane             *p_plane_in,
    int                           roomId_in,
    std::vector<Eigen::Vector3d> &cameraCenters_world_m_out)
{
    cameraCenters_world_m_out.clear();
    std::vector<keyframe::KeyFrame *> observerKeyFrames{};
    if (collectObserverKeyFrames(p_plane_in, observerKeyFrames) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: collectObserverKeyFrames returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    for (keyframe::KeyFrame *p_observer : observerKeyFrames)
    {
        Eigen::Vector3f observerCameraCenter_world_m{};
        if (p_observer->roomId != roomId_in)
        {
            continue;
        }
        if (p_observer->getCameraCenter(observerCameraCenter_world_m) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCameraCenter returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (observerCameraCenter_world_m.allFinite())
        {
            cameraCenters_world_m_out.push_back(
                observerCameraCenter_world_m.cast<double>());
        }
    }
    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
