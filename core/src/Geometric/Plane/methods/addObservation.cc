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

#include "Geometric/Plane.h"
#include "KeyFrame.h"
#include <algorithm>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <limits>
#include <mutex>
#include <pcl/octree/octree_search.h>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

PlaneStatus Plane::addObservation(core::KeyFrame    *p_keyFrame_inout,
                                  const Observation &observation_in)
{
    /* Confirm the keyframe is valid */
    bool keyFrameIsBad{};
    if (!(p_keyFrame_inout == nullptr) &&
        p_keyFrame_inout->isBad(keyFrameIsBad) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_keyFrame_inout == nullptr || keyFrameIsBad)
    {
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    /* Lock the plane observation data */
    unique_lock<mutex> lock(featuresMutex);

    /*!
     * Insert the observation only when the keyframe has not previously
     * observed this plane.
     */
    const auto insertionResult =
        observations.insert({p_keyFrame_inout, observation_in});

    /* Increment the observation count after a successful insertion */
    if (insertionResult.second)
    {
        observationCount++;

        if (p_refKeyFrame == nullptr)
        {
            p_refKeyFrame = p_keyFrame_inout;
        }
    }

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs
