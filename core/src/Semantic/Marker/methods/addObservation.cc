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
 * @file            addObservation.cc
 *
 * @brief           Implements Marker::addObservation(), declared in
 *                  Semantic/Marker.h.
 */

#include "KeyFrame.h"
#include "Semantic/Marker.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

MarkerStatus
    Marker::addObservation(core::KeyFrame     *p_keyFrame_in,
                           const Sophus::SE3f &markerPose_markerToCamera_in)
{
    bool keyFrameIsBad{};
    if (!(p_keyFrame_in == nullptr) &&
        p_keyFrame_in->isBad(keyFrameIsBad) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_keyFrame_in == nullptr || keyFrameIsBad)
    {
        return MarkerStatus::MARKER_STATUS_SUCCESS;
    }

    std::lock_guard<std::mutex> lock(observationsMutex);
    observations.insert_or_assign(p_keyFrame_in, markerPose_markerToCamera_in);

    return MarkerStatus::MARKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
