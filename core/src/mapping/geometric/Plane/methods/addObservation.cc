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
 * @brief           Implements Plane::addObservation(), declared in
 *                  Geometric/Plane.h.
 */

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "mapping/geometric/Plane/objects/PlaneClass.h"
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

PlaneStatus Plane::addObservation(keyframe::KeyFrame    *p_keyFrame_inout,
                                  const Observation &observation_in)
{
    /* Confirm the keyframe is valid */
    bool keyFrameIsBad{};
    if (!(p_keyFrame_inout == nullptr) &&
        p_keyFrame_inout->isBad(keyFrameIsBad) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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

    /* Lock observations and semantics for one consistent vote rebuild. */
    std::scoped_lock lock(featuresMutex, typeMutex);

    const std::map<keyframe::KeyFrame *, Observation>::iterator observationIt =
        observations.find(p_keyFrame_inout);
    if (observationIt == observations.end())
    {
        observations.emplace(p_keyFrame_inout, observation_in);
        observationCount++;

        if (p_refKeyFrame == nullptr)
        {
            p_refKeyFrame = p_keyFrame_inout;
        }
    }
    else
    {
        /*!
         * A second patch from the same keyframe joins that keyframe's
         * record, so the keyframe still casts one vote. Only the label
         * evidence is summed per class; the first patch's geometry and
         * optimiser weight (confidence) stay as they were.
         */
        for (const auto &[semanticType, weight] :
             observation_in.semanticEvidence)
        {
            observationIt->second.semanticEvidence[semanticType] += weight;
        }
    }

    if (rebuildSemanticVotesWithoutLock() != PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: rebuildSemanticVotesWithoutLock returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} /* namespace geometric */
} /* namespace core */
} /* namespace vs_graphs */
